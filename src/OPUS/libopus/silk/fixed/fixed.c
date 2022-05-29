
#include "fixed.h"
#include "../silk.h"

static const int16_t freq_table_Q16[27] = {
    12111, 9804, 8235, 7100, 6239, 5565, 5022, 4575, 4202, 3885, 3612, 3375, 3167, 2984,
    2820,  2674, 2542, 2422, 2313, 2214, 2123, 2038, 1961, 1889, 1822, 1760, 1702,
};
//----------------------------------------------------------------------------------------------------------------------
void silk_apply_sine_window(int16_t px_win[],       /* O    Pointer to windowed signal                               */
                            const int16_t px[],     /* I    Pointer to input signal                                  */
                            const int32_t win_type, /* I    Selects a window type                                    */
                            const int32_t length    /* I    Window length, multiple of 4                             */
) {
    int32_t k, f_Q16, c_Q16;
    int32_t S0_Q16, S1_Q16;

    assert(win_type == 1 || win_type == 2);

    /* Length must be in a range from 16 to 120 and a multiple of 4 */
    assert(length >= 16 && length <= 120);
    assert((length & 3) == 0);

    /* Frequency */
    k = (length >> 2) - 4;
    assert(k >= 0 && k <= 26);
    f_Q16 = (int32_t)freq_table_Q16[k];

    /* Factor used for cosine approximation */
    c_Q16 = silk_SMULWB((int32_t)f_Q16, -f_Q16);
    assert(c_Q16 >= -32768);

    /* initialize state */
    if (win_type == 1) {
        /* start from 0 */
        S0_Q16 = 0;
        /* approximation of sin(f) */
        S1_Q16 = f_Q16 + silk_RSHIFT(length, 3);
    } else {
        /* start from 1 */
        S0_Q16 = ((int32_t)1 << 16);
        /* approximation of cos(f) */
        S1_Q16 = ((int32_t)1 << 16) + silk_RSHIFT(c_Q16, 1) + silk_RSHIFT(length, 4);
    }

    /* Uses the recursive equation:   sin(n*f) = 2 * cos(f) * sin((n-1)*f) - sin((n-2)*f)    */
    /* 4 samples at a time */
    for (k = 0; k < length; k += 4) {
        px_win[k] = (int16_t)silk_SMULWB(silk_RSHIFT(S0_Q16 + S1_Q16, 1), px[k]);
        px_win[k + 1] = (int16_t)silk_SMULWB(S1_Q16, px[k + 1]);
        S0_Q16 = silk_SMULWB(S1_Q16, c_Q16) + silk_LSHIFT(S1_Q16, 1) - S0_Q16 + 1;
        S0_Q16 = silk_min(S0_Q16, ((int32_t)1 << 16));

        px_win[k + 2] = (int16_t)silk_SMULWB(silk_RSHIFT(S0_Q16 + S1_Q16, 1), px[k + 2]);
        px_win[k + 3] = (int16_t)silk_SMULWB(S0_Q16, px[k + 3]);
        S1_Q16 = silk_SMULWB(S0_Q16, c_Q16) + silk_LSHIFT(S0_Q16, 1) - S1_Q16;
        S1_Q16 = silk_min(S1_Q16, ((int32_t)1 << 16));
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Compute autocorrelation */
void silk_autocorr(int32_t *results,                /* O    Result (length correlationCount)      */
                   int32_t *scale,                  /* O    Scaling of the correlation vector     */
                   const int16_t *inputData,        /* I    Input data to correlate               */
                   const int32_t inputDataSize,     /* I    Length of input                       */
                   const int32_t correlationCount,  /* I    Number of correlation taps to compute */
                   int arch                         /* I    Run-time architecture                 */
) {
    int32_t corrCount;
    corrCount = silk_min_int(inputDataSize, correlationCount);
    *scale = _celt_autocorr(inputData, results, NULL, 0, corrCount - 1, inputDataSize, arch);
}
//----------------------------------------------------------------------------------------------------------------------

/* Compute reflection coefficients from input signal */
void silk_burg_modified_c(
    int32_t *res_nrg,             /* O    Residual energy                                             */
    int32_t *res_nrg_Q,           /* O    Residual energy Q value                                     */
    int32_t A_Q16[],              /* O    Prediction coefficients (length order)                      */
    const int16_t x[],            /* I    Input signal, length: nb_subfr * ( D + subfr_length )       */
    const int32_t minInvGain_Q30, /* I    Inverse of max prediction gain                              */
    const int32_t subfr_length,   /* I    Input signal subframe length (incl. D preceding samples)    */
    const int32_t nb_subfr,       /* I    Number of subframes stacked in x                            */
    const int32_t D,              /* I    Order                                                       */
    int arch                      /* I    Run-time architecture                                       */
) {
    int32_t k, n, s, lz, rshifts, reached_max_gain;
    int32_t C0, num, nrg, rc_Q31, invGain_Q30, Atmp_QA, Atmp1, tmp1, tmp2, x1, x2;
    const int16_t *x_ptr;
    int32_t *C_first_row = (int32_t *)malloc(sizeof(int32_t) * SILK_MAX_ORDER_LPC);
    int32_t *C_last_row = (int32_t *)malloc(sizeof(int32_t) * SILK_MAX_ORDER_LPC);
    int32_t *Af_QA = (int32_t *)malloc(sizeof(int32_t) * SILK_MAX_ORDER_LPC);
    int32_t *CAf = (int32_t *)malloc(sizeof(int32_t) * (SILK_MAX_ORDER_LPC + 1));
    int32_t *CAb = (int32_t *)malloc(sizeof(int32_t) * (SILK_MAX_ORDER_LPC + 1));
    int32_t *xcorr = (int32_t *)malloc(sizeof(int32_t) * SILK_MAX_ORDER_LPC);
    int64_t C0_64;
    const uint8_t QA25 = 25;

    assert(subfr_length * nb_subfr <= MAX_FRAME_SIZE);

    /* Compute autocorrelations, added over subframes */
    C0_64 = silk_inner_prod16_aligned_64(x, x, subfr_length * nb_subfr, arch);
    lz = silk_CLZ64(C0_64);
    rshifts = 32 + 1 + N_BITS_HEAD_ROOM - lz;
    if (rshifts > MAX_RSHIFTS) rshifts = MAX_RSHIFTS;
    if (rshifts < MIN_RSHIFTS) rshifts = MIN_RSHIFTS;

    if (rshifts > 0) {
        C0 = (int32_t)silk_RSHIFT64(C0_64, rshifts);
    } else {
        C0 = silk_LSHIFT32((int32_t)C0_64, -rshifts);
    }

    CAb[0] = CAf[0] = C0 + silk_SMMUL(SILK_FIX_CONST(FIND_LPC_COND_FAC, 32), C0) + 1; /* Q(-rshifts) */
    silk_memset(C_first_row, 0, SILK_MAX_ORDER_LPC * sizeof(int32_t));
    if (rshifts > 0) {
        for (s = 0; s < nb_subfr; s++) {
            x_ptr = x + s * subfr_length;
            for (n = 1; n < D + 1; n++) {
                C_first_row[n - 1] += (int32_t)silk_RSHIFT64(
                    silk_inner_prod16_aligned_64(x_ptr, x_ptr + n, subfr_length - n, arch), rshifts);
            }
        }
    } else {
        for (s = 0; s < nb_subfr; s++) {
            int i;
            int32_t d;
            x_ptr = x + s * subfr_length;
            celt_pitch_xcorr(x_ptr, x_ptr + 1, xcorr, subfr_length - D, D, arch);
            for (n = 1; n < D + 1; n++) {
                for (i = n + subfr_length - D, d = 0; i < subfr_length; i++) d = MAC16_16(d, x_ptr[i], x_ptr[i - n]);
                xcorr[n - 1] += d;
            }
            for (n = 1; n < D + 1; n++) {
                C_first_row[n - 1] += silk_LSHIFT32(xcorr[n - 1], -rshifts);
            }
        }
    }
    silk_memcpy(C_last_row, C_first_row, SILK_MAX_ORDER_LPC * sizeof(int32_t));

    /* Initialize */
    CAb[0] = CAf[0] = C0 + silk_SMMUL(SILK_FIX_CONST(FIND_LPC_COND_FAC, 32), C0) + 1; /* Q(-rshifts) */

    invGain_Q30 = (int32_t)1 << 30;
    reached_max_gain = 0;
    for (n = 0; n < D; n++) {
        /* Update first row of correlation matrix (without first element) */
        /* Update last row of correlation matrix (without last element, stored in reversed order) */
        /* Update C * Af */
        /* Update C * flipud(Af) (stored in reversed order) */
        if (rshifts > -2) {
            for (s = 0; s < nb_subfr; s++) {
                x_ptr = x + s * subfr_length;
                x1 = -silk_LSHIFT32((int32_t)x_ptr[n], 16 - rshifts);                    /* Q(16-rshifts) */
                x2 = -silk_LSHIFT32((int32_t)x_ptr[subfr_length - n - 1], 16 - rshifts); /* Q(16-rshifts) */
                tmp1 = silk_LSHIFT32((int32_t)x_ptr[n], QA25 - 16);                        /* Q(QA_-16) */
                tmp2 = silk_LSHIFT32((int32_t)x_ptr[subfr_length - n - 1], QA25 - 16);     /* Q(QA_-16) */
                for (k = 0; k < n; k++) {
                    C_first_row[k] = silk_SMLAWB(C_first_row[k], x1, x_ptr[n - k - 1]);          /* Q( -rshifts ) */
                    C_last_row[k] = silk_SMLAWB(C_last_row[k], x2, x_ptr[subfr_length - n + k]); /* Q( -rshifts ) */
                    Atmp_QA = Af_QA[k];
                    tmp1 = silk_SMLAWB(tmp1, Atmp_QA, x_ptr[n - k - 1]);            /* Q(QA_-16) */
                    tmp2 = silk_SMLAWB(tmp2, Atmp_QA, x_ptr[subfr_length - n + k]); /* Q(QA_-16) */
                }
                tmp1 = silk_LSHIFT32(-tmp1, 32 - QA25 - rshifts); /* Q(16-rshifts) */
                tmp2 = silk_LSHIFT32(-tmp2, 32 - QA25 - rshifts); /* Q(16-rshifts) */
                for (k = 0; k <= n; k++) {
                    CAf[k] = silk_SMLAWB(CAf[k], tmp1, x_ptr[n - k]);                    /* Q( -rshift ) */
                    CAb[k] = silk_SMLAWB(CAb[k], tmp2, x_ptr[subfr_length - n + k - 1]); /* Q( -rshift ) */
                }
            }
        } else {
            for (s = 0; s < nb_subfr; s++) {
                x_ptr = x + s * subfr_length;
                x1 = -silk_LSHIFT32((int32_t)x_ptr[n], -rshifts);                    /* Q( -rshifts ) */
                x2 = -silk_LSHIFT32((int32_t)x_ptr[subfr_length - n - 1], -rshifts); /* Q( -rshifts ) */
                tmp1 = silk_LSHIFT32((int32_t)x_ptr[n], 17);                         /* Q17 */
                tmp2 = silk_LSHIFT32((int32_t)x_ptr[subfr_length - n - 1], 17);      /* Q17 */
                for (k = 0; k < n; k++) {
                    C_first_row[k] = silk_MLA(C_first_row[k], x1, x_ptr[n - k - 1]);          /* Q( -rshifts ) */
                    C_last_row[k] = silk_MLA(C_last_row[k], x2, x_ptr[subfr_length - n + k]); /* Q( -rshifts ) */

                    #pragma GCC diagnostic push
                    #pragma GCC diagnostic ignored "-Wshift-count-negative"
                    Atmp1 = silk_RSHIFT_ROUND(Af_QA[k], QA25 - 17);                             /* Q17 */
                    #pragma GCC diagnostic pop

                    /* We sometimes have get overflows in the multiplications (even beyond +/- 2^32),
                       but they cancel each other and the real result seems to always fit in a 32-bit
                       signed integer. This was determined experimentally, not theoretically (unfortunately). */
                    tmp1 = silk_MLA_ovflw(tmp1, x_ptr[n - k - 1], Atmp1);            /* Q17 */
                    tmp2 = silk_MLA_ovflw(tmp2, x_ptr[subfr_length - n + k], Atmp1); /* Q17 */
                }
                tmp1 = -tmp1; /* Q17 */
                tmp2 = -tmp2; /* Q17 */
                for (k = 0; k <= n; k++) {
                    CAf[k] = silk_SMLAWW(CAf[k], tmp1,
                                         silk_LSHIFT32((int32_t)x_ptr[n - k], -rshifts - 1)); /* Q( -rshift ) */
                    CAb[k] = silk_SMLAWW(
                        CAb[k], tmp2,
                        silk_LSHIFT32((int32_t)x_ptr[subfr_length - n + k - 1], -rshifts - 1)); /* Q( -rshift ) */
                }
            }
        }

        /* Calculate nominator and denominator for the next order reflection (parcor) coefficient */
        tmp1 = C_first_row[n];            /* Q( -rshifts ) */
        tmp2 = C_last_row[n];             /* Q( -rshifts ) */
        num = 0;                          /* Q( -rshifts ) */
        nrg = silk_ADD32(CAb[0], CAf[0]); /* Q( 1-rshifts ) */
        for (k = 0; k < n; k++) {
            Atmp_QA = Af_QA[k];
            lz = silk_CLZ32(silk_abs(Atmp_QA)) - 1;
            lz = silk_min(32 - QA25, lz);
            Atmp1 = silk_LSHIFT32(Atmp_QA, lz); /* Q( QA_ + lz ) */

            tmp1 = silk_ADD_LSHIFT32(tmp1, silk_SMMUL(C_last_row[n - k - 1], Atmp1), 32 - QA25 - lz);  /* Q( -rshifts ) */
            tmp2 = silk_ADD_LSHIFT32(tmp2, silk_SMMUL(C_first_row[n - k - 1], Atmp1), 32 - QA25 - lz); /* Q( -rshifts ) */
            num = silk_ADD_LSHIFT32(num, silk_SMMUL(CAb[n - k], Atmp1), 32 - QA25 - lz);               /* Q( -rshifts ) */
            nrg = silk_ADD_LSHIFT32(nrg, silk_SMMUL(silk_ADD32(CAb[k + 1], CAf[k + 1]), Atmp1),
                                    32 - QA25 - lz); /* Q( 1-rshifts ) */
        }
        CAf[n + 1] = tmp1;            /* Q( -rshifts ) */
        CAb[n + 1] = tmp2;            /* Q( -rshifts ) */
        num = silk_ADD32(num, tmp2);  /* Q( -rshifts ) */
        num = silk_LSHIFT32(-num, 1); /* Q( 1-rshifts ) */

        /* Calculate the next order reflection (parcor) coefficient */
        if (silk_abs(num) < nrg) {
            rc_Q31 = silk_DIV32_varQ(num, nrg, 31);
        } else {
            rc_Q31 = (num > 0) ? silk_int32_MAX : silk_int32_MIN;
        }

        /* Update inverse prediction gain */
        tmp1 = ((int32_t)1 << 30) - silk_SMMUL(rc_Q31, rc_Q31);
        tmp1 = silk_LSHIFT(silk_SMMUL(invGain_Q30, tmp1), 2);
        if (tmp1 <= minInvGain_Q30) {
            /* Max prediction gain exceeded; set reflection coefficient such that max prediction gain is exactly hit */
            tmp2 = ((int32_t)1 << 30) - silk_DIV32_varQ(minInvGain_Q30, invGain_Q30, 30); /* Q30 */
            rc_Q31 = silk_SQRT_APPROX(tmp2);                                              /* Q15 */
            if (rc_Q31 > 0) {
                /* Newton-Raphson iteration */
                rc_Q31 = silk_RSHIFT32(rc_Q31 + silk_DIV32(tmp2, rc_Q31), 1); /* Q15 */
                rc_Q31 = silk_LSHIFT32(rc_Q31, 16);                           /* Q31 */
                if (num < 0) {
                    /* Ensure adjusted reflection coefficients has the original sign */
                    rc_Q31 = -rc_Q31;
                }
            }
            invGain_Q30 = minInvGain_Q30;
            reached_max_gain = 1;
        } else {
            invGain_Q30 = tmp1;
        }

        /* Update the AR coefficients */
        for (k = 0; k < (n + 1) >> 1; k++) {
            tmp1 = Af_QA[k];                                                         /* QA_ */
            tmp2 = Af_QA[n - k - 1];                                                 /* QA_ */
            Af_QA[k] = silk_ADD_LSHIFT32(tmp1, silk_SMMUL(tmp2, rc_Q31), 1);         /* QA_ */
            Af_QA[n - k - 1] = silk_ADD_LSHIFT32(tmp2, silk_SMMUL(tmp1, rc_Q31), 1); /* QA_ */
        }
        Af_QA[n] = silk_RSHIFT32(rc_Q31, 31 - QA25); /* QA_ */

        if (reached_max_gain) {
            /* Reached max prediction gain; set remaining coefficients to zero and exit loop */
            for (k = n + 1; k < D; k++) {
                Af_QA[k] = 0;
            }
            break;
        }

        /* Update C * Af and C * Ab */
        for (k = 0; k <= n + 1; k++) {
            tmp1 = CAf[k];                                                         /* Q( -rshifts ) */
            tmp2 = CAb[n - k + 1];                                                 /* Q( -rshifts ) */
            CAf[k] = silk_ADD_LSHIFT32(tmp1, silk_SMMUL(tmp2, rc_Q31), 1);         /* Q( -rshifts ) */
            CAb[n - k + 1] = silk_ADD_LSHIFT32(tmp2, silk_SMMUL(tmp1, rc_Q31), 1); /* Q( -rshifts ) */
        }
    }

    if (reached_max_gain) {
        for (k = 0; k < D; k++) {
            /* Scale coefficients */
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wshift-count-negative"
            A_Q16[k] = -silk_RSHIFT_ROUND(Af_QA[k], QA25 - 16);
            #pragma GCC diagnostic pop
        }
        /* Subtract energy of preceding samples from C0 */
        if (rshifts > 0) {
            for (s = 0; s < nb_subfr; s++) {
                x_ptr = x + s * subfr_length;
                C0 -= (int32_t)silk_RSHIFT64(silk_inner_prod16_aligned_64(x_ptr, x_ptr, D, arch), rshifts);
            }
        } else {
            for (s = 0; s < nb_subfr; s++) {
                x_ptr = x + s * subfr_length;
                C0 -= silk_LSHIFT32(silk_inner_prod_aligned(x_ptr, x_ptr, D, arch), -rshifts);
            }
        }
        /* Approximate residual energy */
        *res_nrg = silk_LSHIFT(silk_SMMUL(invGain_Q30, C0), 2);
        *res_nrg_Q = -rshifts;
    } else {
        /* Return residual energy */
        nrg = CAf[0];            /* Q( -rshifts ) */
        tmp1 = (int32_t)1 << 16; /* Q16 */
        for (k = 0; k < D; k++) {

            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wshift-count-negative"
            Atmp1 = silk_RSHIFT_ROUND(Af_QA[k], QA25 - 16); /* Q16 */
            #pragma GCC diagnostic pop

            nrg = silk_SMLAWW(nrg, CAf[k + 1], Atmp1);    /* Q( -rshifts ) */
            tmp1 = silk_SMLAWW(tmp1, Atmp1, Atmp1);       /* Q16 */
            A_Q16[k] = -Atmp1;
        }
        *res_nrg = silk_SMLAWW(nrg, silk_SMMUL(SILK_FIX_CONST(FIND_LPC_COND_FAC, 32), C0), -tmp1); /* Q( -rshifts ) */
        *res_nrg_Q = -rshifts;
    }
    free(C_first_row);
    free(C_last_row);
    free(Af_QA);
    free(CAf);
    free(CAb);
    free(xcorr);
}
//----------------------------------------------------------------------------------------------------------------------

/* Calculates correlation vector X'*t */
void silk_corrVector_FIX(const int16_t *x,      /* I    x vector [L + order - 1] used to form data matrix X */
                         const int16_t *t,      /* I    Target vector [L]      */
                         const int32_t L,       /* I    Length of vectors       */
                         const int32_t order,   /* I    Max lag for correlation   */
                         int32_t *Xt,           /* O    Pointer to X'*t correlation vector [order]          */
                         const int32_t rshifts, /* I    Right shifts of correlations */
                         int arch /* I    Run-time architecture                                             */
) {
    int32_t lag, i;
    const int16_t *ptr1, *ptr2;
    int32_t inner_prod;

    ptr1 = &x[order - 1]; /* Points to first sample of column 0 of X: X[:,0] */
    ptr2 = t;
    /* Calculate X'*t */
    if (rshifts > 0) {
        /* Right shifting used */
        for (lag = 0; lag < order; lag++) {
            inner_prod = 0;
            for (i = 0; i < L; i++) {
                inner_prod = silk_ADD_RSHIFT32(inner_prod, silk_SMULBB(ptr1[i], ptr2[i]), rshifts);
            }
            Xt[lag] = inner_prod; /* X[:,lag]'*t */
            ptr1--;               /* Go to next column of X */
        }
    } else {
        assert(rshifts == 0);
        for (lag = 0; lag < order; lag++) {
            Xt[lag] = silk_inner_prod_aligned(ptr1, ptr2, L, arch); /* X[:,lag]'*t */
            ptr1--;                                                 /* Go to next column of X */
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Calculates correlation matrix X'*X */
void silk_corrMatrix_FIX(const int16_t *x,    /* I    x vector [L + order - 1] used to form data matrix X    */
                         const int32_t L,     /* I    Length of vectors     */
                         const int32_t order, /* I    Max lag for correlation */
                         int32_t *XX,         /* O    Pointer to X'*X correlation matrix [ order x order ]   */
                         int32_t *nrg,        /* O    Energy of x vector        */
                         int32_t *rshifts,    /* O    Right shifts of correlations and energy    */
                         int arch /* I    Run-time architecture                                              */
) {
    int32_t i, j, lag;
    int32_t energy;
    const int16_t *ptr1, *ptr2;

    /* Calculate energy to find shift used to fit in 32 bits */
    silk_sum_sqr_shift(nrg, rshifts, x, L + order - 1);
    energy = *nrg;

    /* Calculate energy of first column (0) of X: X[:,0]'*X[:,0] */
    /* Remove contribution of first order - 1 samples */
    for (i = 0; i < order - 1; i++) {
        energy -= silk_RSHIFT32(silk_SMULBB(x[i], x[i]), *rshifts);
    }

    /* Calculate energy of remaining columns of X: X[:,j]'*X[:,j] */
    /* Fill out the diagonal of the correlation matrix */
    matrix_ptr(XX, 0, 0, order) = energy;
    assert(energy >= 0);
    ptr1 = &x[order - 1]; /* First sample of column 0 of X */
    for (j = 1; j < order; j++) {
        energy = silk_SUB32(energy, silk_RSHIFT32(silk_SMULBB(ptr1[L - j], ptr1[L - j]), *rshifts));
        energy = silk_ADD32(energy, silk_RSHIFT32(silk_SMULBB(ptr1[-j], ptr1[-j]), *rshifts));
        matrix_ptr(XX, j, j, order) = energy;
        assert(energy >= 0);
    }

    ptr2 = &x[order - 2]; /* First sample of column 1 of X */
    /* Calculate the remaining elements of the correlation matrix */
    if (*rshifts > 0) {
        /* Right shifting used */
        for (lag = 1; lag < order; lag++) {
            /* Inner product of column 0 and column lag: X[:,0]'*X[:,lag] */
            energy = 0;
            for (i = 0; i < L; i++) {
                energy += silk_RSHIFT32(silk_SMULBB(ptr1[i], ptr2[i]), *rshifts);
            }
            /* Calculate remaining off diagonal: X[:,j]'*X[:,j + lag] */
            matrix_ptr(XX, lag, 0, order) = energy;
            matrix_ptr(XX, 0, lag, order) = energy;
            for (j = 1; j < (order - lag); j++) {
                energy = silk_SUB32(energy, silk_RSHIFT32(silk_SMULBB(ptr1[L - j], ptr2[L - j]), *rshifts));
                energy = silk_ADD32(energy, silk_RSHIFT32(silk_SMULBB(ptr1[-j], ptr2[-j]), *rshifts));
                matrix_ptr(XX, lag + j, j, order) = energy;
                matrix_ptr(XX, j, lag + j, order) = energy;
            }
            ptr2--; /* Update pointer to first sample of next column (lag) in X */
        }
    } else {
        for (lag = 1; lag < order; lag++) {
            /* Inner product of column 0 and column lag: X[:,0]'*X[:,lag] */
            energy = silk_inner_prod_aligned(ptr1, ptr2, L, arch);
            matrix_ptr(XX, lag, 0, order) = energy;
            matrix_ptr(XX, 0, lag, order) = energy;
            /* Calculate remaining off diagonal: X[:,j]'*X[:,j + lag] */
            for (j = 1; j < (order - lag); j++) {
                energy = silk_SUB32(energy, silk_SMULBB(ptr1[L - j], ptr2[L - j]));
                energy = silk_SMLABB(energy, ptr1[-j], ptr2[-j]);
                matrix_ptr(XX, lag + j, j, order) = energy;
                matrix_ptr(XX, j, lag + j, order) = energy;
            }
            ptr2--; /* Update pointer to first sample of next column (lag) in X */
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Low Bitrate Redundancy (LBRR) encoding. Reuse all parameters but encode with lower bitrate              */
static inline void silk_LBRR_encode_FIX(
    silk_encoder_state_FIX *psEnc,       /* I/O  Pointer to Silk FIX encoder state       */
    silk_encoder_control_FIX *psEncCtrl, /* I/O  Pointer to Silk FIX encoder control struct */
    const int16_t x16[], /* I    Input signal                                                              */
    int32_t condCoding   /* I    The type of conditional coding used so far for this frame                 */
);

void silk_encode_do_VAD_FIX(silk_encoder_state_FIX *psEnc, /* I/O  Pointer to Silk FIX encoder state        */
                            int32_t activity               /* I    Decision of Opus voice activity detector */
) {
    const int32_t activity_threshold = SILK_FIX_CONST(SPEECH_ACTIVITY_DTX_THRES, 8);

    /****************************/
    /* Voice Activity Detection */
    /****************************/
    silk_VAD_GetSA_Q8(&psEnc->sCmn, psEnc->sCmn.inputBuf + 1, psEnc->sCmn.arch);
    /* If Opus VAD is inactive and Silk VAD is active: lower Silk VAD to just under the threshold */
    if (activity == VAD_NO_ACTIVITY && psEnc->sCmn.speech_activity_Q8 >= activity_threshold) {
        psEnc->sCmn.speech_activity_Q8 = activity_threshold - 1;
    }

    /**************************************************/
    /* Convert speech activity into VAD and DTX flags */
    /**************************************************/
    if (psEnc->sCmn.speech_activity_Q8 < activity_threshold) {
        psEnc->sCmn.indices.signalType = TYPE_NO_VOICE_ACTIVITY;
        psEnc->sCmn.noSpeechCounter++;
        if (psEnc->sCmn.noSpeechCounter <= NB_SPEECH_FRAMES_BEFORE_DTX) {
            psEnc->sCmn.inDTX = 0;
        } else if (psEnc->sCmn.noSpeechCounter > MAX_CONSECUTIVE_DTX + NB_SPEECH_FRAMES_BEFORE_DTX) {
            psEnc->sCmn.noSpeechCounter = NB_SPEECH_FRAMES_BEFORE_DTX;
            psEnc->sCmn.inDTX = 0;
        }
        psEnc->sCmn.VAD_flags[psEnc->sCmn.nFramesEncoded] = 0;
    } else {
        psEnc->sCmn.noSpeechCounter = 0;
        psEnc->sCmn.inDTX = 0;
        psEnc->sCmn.indices.signalType = TYPE_UNVOICED;
        psEnc->sCmn.VAD_flags[psEnc->sCmn.nFramesEncoded] = 1;
    }
}
//----------------------------------------------------------------------------------------------------------------------
/* Encode frame */
int32_t silk_encode_frame_FIX(silk_encoder_state_FIX *psEnc, /* I/O  Pointer to Silk FIX encoder state */
                              int32_t *pnBytesOut,           /* O    Pointer to number of payload bytes;        */
                              ec_enc *psRangeEnc,            /* I/O  compressor data structure            */
                              int32_t condCoding,            /* I    The type of conditional coding to use      */
                              int32_t maxBits,               /* I    If > 0: maximum number of output bits      */
                              int32_t useCBR                 /* I    Flag to force constant-bitrate operation   */
) {
    silk_encoder_control_FIX sEncCtrl;
    int32_t i, iter, maxIter, found_upper, found_lower, ret = 0;
    int16_t *x_frame;
    ec_enc sRangeEnc_copy, sRangeEnc_copy2;
    silk_nsq_state sNSQ_copy, sNSQ_copy2;
    int32_t seed_copy, nBits, nBits_lower, nBits_upper, gainMult_lower, gainMult_upper;
    int32_t gainsID, gainsID_lower, gainsID_upper;
    int16_t gainMult_Q8;
    int16_t ec_prevLagIndex_copy;
    int32_t ec_prevSignalType_copy;
    int8_t LastGainIndex_copy2;
    int32_t gain_lock[MAX_NB_SUBFR] = {0};
    int16_t best_gain_mult[MAX_NB_SUBFR];
    int32_t best_sum[MAX_NB_SUBFR];
    SAVE_STACK;

    /* This is totally unnecessary but many compilers (including gcc) are too dumb to realise it */
    LastGainIndex_copy2 = nBits_lower = nBits_upper = gainMult_lower = gainMult_upper = 0;

    psEnc->sCmn.indices.Seed = psEnc->sCmn.frameCounter++ & 3;

    /**************************************************************/
    /* Set up Input Pointers, and insert frame in input buffer   */
    /*************************************************************/
    /* start of frame to encode */
    x_frame = psEnc->x_buf + psEnc->sCmn.ltp_mem_length;

    /***************************************/
    /* Ensure smooth bandwidth transitions */
    /***************************************/
    silk_LP_variable_cutoff(&psEnc->sCmn.sLP, psEnc->sCmn.inputBuf + 1, psEnc->sCmn.frame_length);

    /*******************************************/
    /* Copy new frame to front of input buffer */
    /*******************************************/
    silk_memcpy(x_frame + LA_SHAPE_MS * psEnc->sCmn.fs_kHz, psEnc->sCmn.inputBuf + 1,
                psEnc->sCmn.frame_length * sizeof(int16_t));

    if (!psEnc->sCmn.prefillFlag) {
        VARDECL(int16_t, res_pitch);
        VARDECL(uint8_t, ec_buf_copy);
        int16_t *res_pitch_frame;

        ALLOC(res_pitch, psEnc->sCmn.la_pitch + psEnc->sCmn.frame_length + psEnc->sCmn.ltp_mem_length, int16_t);
        /* start of pitch LPC residual frame */
        res_pitch_frame = res_pitch + psEnc->sCmn.ltp_mem_length;

        /* Find pitch lags, initial LPC analysis */
        silk_find_pitch_lags_FIX(psEnc, &sEncCtrl, res_pitch, x_frame - psEnc->sCmn.ltp_mem_length, psEnc->sCmn.arch);

        /* Noise shape analysis */
        silk_noise_shape_analysis_FIX(psEnc, &sEncCtrl, res_pitch_frame, x_frame, psEnc->sCmn.arch);

        /* Find linear prediction coefficients (LPC + LTP) */
        silk_find_pred_coefs_FIX(psEnc, &sEncCtrl, res_pitch_frame, x_frame, condCoding);

        /****************************************/
        /* Process gains                        */
        /****************************************/
        silk_process_gains_FIX(psEnc, &sEncCtrl, condCoding);

        /****************************************/
        /* Low Bitrate Redundant Encoding       */
        /****************************************/
        silk_LBRR_encode_FIX(psEnc, &sEncCtrl, x_frame, condCoding);

        /* Loop over quantizer and entropy coding to control bitrate */
        maxIter = 6;
        gainMult_Q8 = SILK_FIX_CONST(1, 8);
        found_lower = 0;
        found_upper = 0;
        gainsID = silk_gains_ID(psEnc->sCmn.indices.GainsIndices, psEnc->sCmn.nb_subfr);
        gainsID_lower = -1;
        gainsID_upper = -1;
        /* Copy part of the input state */
        silk_memcpy(&sRangeEnc_copy, psRangeEnc, sizeof(ec_enc));
        silk_memcpy(&sNSQ_copy, &psEnc->sCmn.sNSQ, sizeof(silk_nsq_state));
        seed_copy = psEnc->sCmn.indices.Seed;
        ec_prevLagIndex_copy = psEnc->sCmn.ec_prevLagIndex;
        ec_prevSignalType_copy = psEnc->sCmn.ec_prevSignalType;
        ALLOC(ec_buf_copy, 1275, uint8_t);
        for (iter = 0;; iter++) {
            if (gainsID == gainsID_lower) {
                nBits = nBits_lower;
            } else if (gainsID == gainsID_upper) {
                nBits = nBits_upper;
            } else {
                /* Restore part of the input state */
                if (iter > 0) {
                    silk_memcpy(psRangeEnc, &sRangeEnc_copy, sizeof(ec_enc));
                    silk_memcpy(&psEnc->sCmn.sNSQ, &sNSQ_copy, sizeof(silk_nsq_state));
                    psEnc->sCmn.indices.Seed = seed_copy;
                    psEnc->sCmn.ec_prevLagIndex = ec_prevLagIndex_copy;
                    psEnc->sCmn.ec_prevSignalType = ec_prevSignalType_copy;
                }

                /*****************************************/
                /* Noise shaping quantization            */
                /*****************************************/
                if (psEnc->sCmn.nStatesDelayedDecision > 1 || psEnc->sCmn.warping_Q16 > 0) {
                    silk_NSQ_del_dec(&psEnc->sCmn, &psEnc->sCmn.sNSQ, &psEnc->sCmn.indices, x_frame, psEnc->sCmn.pulses,
                                     sEncCtrl.PredCoef_Q12[0], sEncCtrl.LTPCoef_Q14, sEncCtrl.AR_Q13,
                                     sEncCtrl.HarmShapeGain_Q14, sEncCtrl.Tilt_Q14, sEncCtrl.LF_shp_Q14,
                                     sEncCtrl.Gains_Q16, sEncCtrl.pitchL, sEncCtrl.Lambda_Q10, sEncCtrl.LTP_scale_Q14,
                                     psEnc->sCmn.arch);
                } else {
                    silk_NSQ(&psEnc->sCmn, &psEnc->sCmn.sNSQ, &psEnc->sCmn.indices, x_frame, psEnc->sCmn.pulses,
                             sEncCtrl.PredCoef_Q12[0], sEncCtrl.LTPCoef_Q14, sEncCtrl.AR_Q13,
                             sEncCtrl.HarmShapeGain_Q14, sEncCtrl.Tilt_Q14, sEncCtrl.LF_shp_Q14, sEncCtrl.Gains_Q16,
                             sEncCtrl.pitchL, sEncCtrl.Lambda_Q10, sEncCtrl.LTP_scale_Q14, psEnc->sCmn.arch);
                }

                if (iter == maxIter && !found_lower) {
                    silk_memcpy(&sRangeEnc_copy2, psRangeEnc, sizeof(ec_enc));
                }

                /****************************************/
                /* Encode Parameters                    */
                /****************************************/
                silk_encode_indices(&psEnc->sCmn, psRangeEnc, psEnc->sCmn.nFramesEncoded, 0, condCoding);

                /****************************************/
                /* Encode Excitation Signal             */
                /****************************************/
                silk_encode_pulses(psRangeEnc, psEnc->sCmn.indices.signalType, psEnc->sCmn.indices.quantOffsetType,
                                   psEnc->sCmn.pulses, psEnc->sCmn.frame_length);

                nBits = ec_tell(psRangeEnc);

                /* If we still bust after the last iteration, do some damage control. */
                if (iter == maxIter && !found_lower && nBits > maxBits) {
                    silk_memcpy(psRangeEnc, &sRangeEnc_copy2, sizeof(ec_enc));

                    /* Keep gains the same as the last frame. */
                    psEnc->sShape.LastGainIndex = sEncCtrl.lastGainIndexPrev;
                    for (i = 0; i < psEnc->sCmn.nb_subfr; i++) {
                        psEnc->sCmn.indices.GainsIndices[i] = 4;
                    }
                    if (condCoding != CODE_CONDITIONALLY) {
                        psEnc->sCmn.indices.GainsIndices[0] = sEncCtrl.lastGainIndexPrev;
                    }
                    psEnc->sCmn.ec_prevLagIndex = ec_prevLagIndex_copy;
                    psEnc->sCmn.ec_prevSignalType = ec_prevSignalType_copy;
                    /* Clear all pulses. */
                    for (i = 0; i < psEnc->sCmn.frame_length; i++) {
                        psEnc->sCmn.pulses[i] = 0;
                    }

                    silk_encode_indices(&psEnc->sCmn, psRangeEnc, psEnc->sCmn.nFramesEncoded, 0, condCoding);

                    silk_encode_pulses(psRangeEnc, psEnc->sCmn.indices.signalType, psEnc->sCmn.indices.quantOffsetType,
                                       psEnc->sCmn.pulses, psEnc->sCmn.frame_length);

                    nBits = ec_tell(psRangeEnc);
                }

                if (useCBR == 0 && iter == 0 && nBits <= maxBits) {
                    break;
                }
            }

            if (iter == maxIter) {
                if (found_lower && (gainsID == gainsID_lower || nBits > maxBits)) {
                    /* Restore output state from earlier iteration that did meet the bitrate budget */
                    silk_memcpy(psRangeEnc, &sRangeEnc_copy2, sizeof(ec_enc));
                    assert(sRangeEnc_copy2.offs <= 1275);
                    silk_memcpy(psRangeEnc->buf, ec_buf_copy, sRangeEnc_copy2.offs);
                    silk_memcpy(&psEnc->sCmn.sNSQ, &sNSQ_copy2, sizeof(silk_nsq_state));
                    psEnc->sShape.LastGainIndex = LastGainIndex_copy2;
                }
                break;
            }

            if (nBits > maxBits) {
                if (found_lower == 0 && iter >= 2) {
                    /* Adjust the quantizer's rate/distortion tradeoff and discard previous "upper" results */
                    sEncCtrl.Lambda_Q10 = silk_ADD_RSHIFT32(sEncCtrl.Lambda_Q10, sEncCtrl.Lambda_Q10, 1);
                    found_upper = 0;
                    gainsID_upper = -1;
                } else {
                    found_upper = 1;
                    nBits_upper = nBits;
                    gainMult_upper = gainMult_Q8;
                    gainsID_upper = gainsID;
                }
            } else if (nBits < maxBits - 5) {
                found_lower = 1;
                nBits_lower = nBits;
                gainMult_lower = gainMult_Q8;
                if (gainsID != gainsID_lower) {
                    gainsID_lower = gainsID;
                    /* Copy part of the output state */
                    silk_memcpy(&sRangeEnc_copy2, psRangeEnc, sizeof(ec_enc));
                    assert(psRangeEnc->offs <= 1275);
                    silk_memcpy(ec_buf_copy, psRangeEnc->buf, psRangeEnc->offs);
                    silk_memcpy(&sNSQ_copy2, &psEnc->sCmn.sNSQ, sizeof(silk_nsq_state));
                    LastGainIndex_copy2 = psEnc->sShape.LastGainIndex;
                }
            } else {
                /* Within 5 bits of budget: close enough */
                break;
            }

            if (!found_lower && nBits > maxBits) {
                int j;
                for (i = 0; i < psEnc->sCmn.nb_subfr; i++) {
                    int sum = 0;
                    for (j = i * psEnc->sCmn.subfr_length; j < (i + 1) * psEnc->sCmn.subfr_length; j++) {
                        sum += abs(psEnc->sCmn.pulses[j]);
                    }
                    if (iter == 0 || (sum < best_sum[i] && !gain_lock[i])) {
                        best_sum[i] = sum;
                        best_gain_mult[i] = gainMult_Q8;
                    } else {
                        gain_lock[i] = 1;
                    }
                }
            }
            if ((found_lower & found_upper) == 0) {
                /* Adjust gain according to high-rate rate/distortion curve */
                if (nBits > maxBits) {
                    if (gainMult_Q8 < 16384) {
                        gainMult_Q8 *= 2;
                    } else {
                        gainMult_Q8 = 32767;
                    }
                } else {
                    int32_t gain_factor_Q16;
                    gain_factor_Q16 = silk_log2lin(silk_LSHIFT(nBits - maxBits, 7) / psEnc->sCmn.frame_length +
                                                   SILK_FIX_CONST(16, 7));
                    gainMult_Q8 = silk_SMULWB(gain_factor_Q16, gainMult_Q8);
                }

            } else {
                /* Adjust gain by interpolating */
                gainMult_Q8 =
                    gainMult_lower + silk_DIV32_16(silk_MUL(gainMult_upper - gainMult_lower, maxBits - nBits_lower),
                                                   nBits_upper - nBits_lower);
                /* New gain multplier must be between 25% and 75% of old range (note that gainMult_upper <
                 * gainMult_lower) */
                if (gainMult_Q8 > silk_ADD_RSHIFT32(gainMult_lower, gainMult_upper - gainMult_lower, 2)) {
                    gainMult_Q8 = silk_ADD_RSHIFT32(gainMult_lower, gainMult_upper - gainMult_lower, 2);
                } else if (gainMult_Q8 < silk_SUB_RSHIFT32(gainMult_upper, gainMult_upper - gainMult_lower, 2)) {
                    gainMult_Q8 = silk_SUB_RSHIFT32(gainMult_upper, gainMult_upper - gainMult_lower, 2);
                }
            }

            for (i = 0; i < psEnc->sCmn.nb_subfr; i++) {
                int16_t tmp;
                if (gain_lock[i]) {
                    tmp = best_gain_mult[i];
                } else {
                    tmp = gainMult_Q8;
                }
                sEncCtrl.Gains_Q16[i] = silk_LSHIFT_SAT32(silk_SMULWB(sEncCtrl.GainsUnq_Q16[i], tmp), 8);
            }

            /* Quantize gains */
            psEnc->sShape.LastGainIndex = sEncCtrl.lastGainIndexPrev;
            silk_gains_quant(psEnc->sCmn.indices.GainsIndices, sEncCtrl.Gains_Q16, &psEnc->sShape.LastGainIndex,
                             condCoding == CODE_CONDITIONALLY, psEnc->sCmn.nb_subfr);

            /* Unique identifier of gains vector */
            gainsID = silk_gains_ID(psEnc->sCmn.indices.GainsIndices, psEnc->sCmn.nb_subfr);
        }
    }

    /* Update input buffer */
    silk_memmove(psEnc->x_buf, &psEnc->x_buf[psEnc->sCmn.frame_length],
                 (psEnc->sCmn.ltp_mem_length + LA_SHAPE_MS * psEnc->sCmn.fs_kHz) * sizeof(int16_t));

    /* Exit without entropy coding */
    if (psEnc->sCmn.prefillFlag) {
        /* No payload */
        *pnBytesOut = 0;

        return ret;
    }

    /* Parameters needed for next frame */
    psEnc->sCmn.prevLag = sEncCtrl.pitchL[psEnc->sCmn.nb_subfr - 1];
    psEnc->sCmn.prevSignalType = psEnc->sCmn.indices.signalType;

    /****************************************/
    /* Finalize payload                     */
    /****************************************/
    psEnc->sCmn.first_frame_after_reset = 0;
    /* Payload size */
    *pnBytesOut = silk_RSHIFT(ec_tell(psRangeEnc) + 7, 3);

    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

/* Low-Bitrate Redundancy (LBRR) encoding. Reuse all parameters but encode excitation at lower bitrate  */
static void silk_LBRR_encode_FIX(
    silk_encoder_state_FIX *psEnc,       /* I/O  Pointer to Silk FIX encoder state       */
    silk_encoder_control_FIX *psEncCtrl, /* I/O  Pointer to Silk FIX encoder control struct */
    const int16_t x16[], /* I    Input signal                                                           */
    int32_t condCoding   /* I    The type of conditional coding used so far for this frame              */
) {
    int32_t TempGains_Q16[MAX_NB_SUBFR];
    SideInfoIndices *psIndices_LBRR = &psEnc->sCmn.indices_LBRR[psEnc->sCmn.nFramesEncoded];
    silk_nsq_state sNSQ_LBRR;

    /*******************************************/
    /* Control use of inband LBRR              */
    /*******************************************/
    if (psEnc->sCmn.LBRR_enabled && psEnc->sCmn.speech_activity_Q8 > SILK_FIX_CONST(LBRR_SPEECH_ACTIVITY_THRES, 8)) {
        psEnc->sCmn.LBRR_flags[psEnc->sCmn.nFramesEncoded] = 1;

        /* Copy noise shaping quantizer state and quantization indices from regular encoding */
        silk_memcpy(&sNSQ_LBRR, &psEnc->sCmn.sNSQ, sizeof(silk_nsq_state));
        silk_memcpy(psIndices_LBRR, &psEnc->sCmn.indices, sizeof(SideInfoIndices));

        /* Save original gains */
        silk_memcpy(TempGains_Q16, psEncCtrl->Gains_Q16, psEnc->sCmn.nb_subfr * sizeof(int32_t));

        if (psEnc->sCmn.nFramesEncoded == 0 || psEnc->sCmn.LBRR_flags[psEnc->sCmn.nFramesEncoded - 1] == 0) {
            /* First frame in packet or previous frame not LBRR coded */
            psEnc->sCmn.LBRRprevLastGainIndex = psEnc->sShape.LastGainIndex;

            /* Increase Gains to get target LBRR rate */
            psIndices_LBRR->GainsIndices[0] = psIndices_LBRR->GainsIndices[0] + psEnc->sCmn.LBRR_GainIncreases;
            psIndices_LBRR->GainsIndices[0] = silk_min_int(psIndices_LBRR->GainsIndices[0], N_LEVELS_QGAIN - 1);
        }

        /* Decode to get gains in sync with decoder         */
        /* Overwrite unquantized gains with quantized gains */
        silk_gains_dequant(psEncCtrl->Gains_Q16, psIndices_LBRR->GainsIndices, &psEnc->sCmn.LBRRprevLastGainIndex,
                           condCoding == CODE_CONDITIONALLY, psEnc->sCmn.nb_subfr);

        /*****************************************/
        /* Noise shaping quantization            */
        /*****************************************/
        if (psEnc->sCmn.nStatesDelayedDecision > 1 || psEnc->sCmn.warping_Q16 > 0) {
            silk_NSQ_del_dec(&psEnc->sCmn, &sNSQ_LBRR, psIndices_LBRR, x16,
                             psEnc->sCmn.pulses_LBRR[psEnc->sCmn.nFramesEncoded], psEncCtrl->PredCoef_Q12[0],
                             psEncCtrl->LTPCoef_Q14, psEncCtrl->AR_Q13, psEncCtrl->HarmShapeGain_Q14,
                             psEncCtrl->Tilt_Q14, psEncCtrl->LF_shp_Q14, psEncCtrl->Gains_Q16, psEncCtrl->pitchL,
                             psEncCtrl->Lambda_Q10, psEncCtrl->LTP_scale_Q14, psEnc->sCmn.arch);
        } else {
            silk_NSQ(&psEnc->sCmn, &sNSQ_LBRR, psIndices_LBRR, x16, psEnc->sCmn.pulses_LBRR[psEnc->sCmn.nFramesEncoded],
                     psEncCtrl->PredCoef_Q12[0], psEncCtrl->LTPCoef_Q14, psEncCtrl->AR_Q13,
                     psEncCtrl->HarmShapeGain_Q14, psEncCtrl->Tilt_Q14, psEncCtrl->LF_shp_Q14, psEncCtrl->Gains_Q16,
                     psEncCtrl->pitchL, psEncCtrl->Lambda_Q10, psEncCtrl->LTP_scale_Q14, psEnc->sCmn.arch);
        }

        /* Restore original gains */
        silk_memcpy(psEncCtrl->Gains_Q16, TempGains_Q16, psEnc->sCmn.nb_subfr * sizeof(int32_t));
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Finds LPC vector from correlations, and converts to NLSF */
void silk_find_LPC_FIX(silk_encoder_state *psEncC,  /* I/O  Encoder state  */
                       int16_t NLSF_Q15[],          /* O    NLSFs          */
                       const int16_t x[],           /* I    Input signal           */
                       const int32_t minInvGain_Q30 /* I    Inverse of max prediction gain */
) {
    int32_t k, subfr_length;
    int32_t a_Q16[MAX_LPC_ORDER];
    int32_t isInterpLower, shift;
    int32_t res_nrg0, res_nrg1;
    int32_t rshift0, rshift1;

    /* Used only for LSF interpolation */
    int32_t a_tmp_Q16[MAX_LPC_ORDER], res_nrg_interp, res_nrg, res_tmp_nrg;
    int32_t res_nrg_interp_Q, res_nrg_Q, res_tmp_nrg_Q;
    int16_t a_tmp_Q12[MAX_LPC_ORDER];
    int16_t NLSF0_Q15[MAX_LPC_ORDER];
    SAVE_STACK;

    subfr_length = psEncC->subfr_length + psEncC->predictLPCOrder;

    /* Default: no interpolation */
    psEncC->indices.NLSFInterpCoef_Q2 = 4;

    /* Burg AR analysis for the full frame */
    silk_burg_modified(&res_nrg, &res_nrg_Q, a_Q16, x, minInvGain_Q30, subfr_length, psEncC->nb_subfr,
                       psEncC->predictLPCOrder, psEncC->arch);

    if (psEncC->useInterpolatedNLSFs && !psEncC->first_frame_after_reset && psEncC->nb_subfr == MAX_NB_SUBFR) {
        VARDECL(int16_t, LPC_res);

        /* Optimal solution for last 10 ms */
        silk_burg_modified(&res_tmp_nrg, &res_tmp_nrg_Q, a_tmp_Q16, x + 2 * subfr_length, minInvGain_Q30, subfr_length,
                           2, psEncC->predictLPCOrder, psEncC->arch);

        /* subtract residual energy here, as that's easier than adding it to the    */
        /* residual energy of the first 10 ms in each iteration of the search below */
        shift = res_tmp_nrg_Q - res_nrg_Q;
        if (shift >= 0) {
            if (shift < 32) {
                res_nrg = res_nrg - silk_RSHIFT(res_tmp_nrg, shift);
            }
        } else {
            assert(shift > -32);
            res_nrg = silk_RSHIFT(res_nrg, -shift) - res_tmp_nrg;
            res_nrg_Q = res_tmp_nrg_Q;
        }

        /* Convert to NLSFs */
        silk_A2NLSF(NLSF_Q15, a_tmp_Q16, psEncC->predictLPCOrder);

        ALLOC(LPC_res, 2 * subfr_length, int16_t);

        /* Search over interpolation indices to find the one with lowest residual energy */
        for (k = 3; k >= 0; k--) {
            /* Interpolate NLSFs for first half */
            silk_interpolate(NLSF0_Q15, psEncC->prev_NLSFq_Q15, NLSF_Q15, k, psEncC->predictLPCOrder);

            /* Convert to LPC for residual energy evaluation */
            silk_NLSF2A(a_tmp_Q12, NLSF0_Q15, psEncC->predictLPCOrder, psEncC->arch);

            /* Calculate residual energy with NLSF interpolation */
            silk_LPC_analysis_filter(LPC_res, x, a_tmp_Q12, 2 * subfr_length, psEncC->predictLPCOrder, psEncC->arch);

            silk_sum_sqr_shift(&res_nrg0, &rshift0, LPC_res + psEncC->predictLPCOrder,
                               subfr_length - psEncC->predictLPCOrder);
            silk_sum_sqr_shift(&res_nrg1, &rshift1, LPC_res + psEncC->predictLPCOrder + subfr_length,
                               subfr_length - psEncC->predictLPCOrder);

            /* Add subframe energies from first half frame */
            shift = rshift0 - rshift1;
            if (shift >= 0) {
                res_nrg1 = silk_RSHIFT(res_nrg1, shift);
                res_nrg_interp_Q = -rshift0;
            } else {
                res_nrg0 = silk_RSHIFT(res_nrg0, -shift);
                res_nrg_interp_Q = -rshift1;
            }
            res_nrg_interp = silk_ADD32(res_nrg0, res_nrg1);

            /* Compare with first half energy without NLSF interpolation, or best interpolated value so far */
            shift = res_nrg_interp_Q - res_nrg_Q;
            if (shift >= 0) {
                if (silk_RSHIFT(res_nrg_interp, shift) < res_nrg) {
                    isInterpLower = silk_TRUE;
                } else {
                    isInterpLower = silk_FALSE;
                }
            } else {
                if (-shift < 32) {
                    if (res_nrg_interp < silk_RSHIFT(res_nrg, -shift)) {
                        isInterpLower = silk_TRUE;
                    } else {
                        isInterpLower = silk_FALSE;
                    }
                } else {
                    isInterpLower = silk_FALSE;
                }
            }

            /* Determine whether current interpolated NLSFs are best so far */
            if (isInterpLower == silk_TRUE) {
                /* Interpolation has lower residual energy */
                res_nrg = res_nrg_interp;
                res_nrg_Q = res_nrg_interp_Q;
                psEncC->indices.NLSFInterpCoef_Q2 = (int8_t)k;
            }
        }
    }

    if (psEncC->indices.NLSFInterpCoef_Q2 == 4) {
        /* NLSF interpolation is currently inactive, calculate NLSFs from full frame AR coefficients */
        silk_A2NLSF(NLSF_Q15, a_Q16, psEncC->predictLPCOrder);
    }

    assert(psEncC->indices.NLSFInterpCoef_Q2 == 4 ||
           (psEncC->useInterpolatedNLSFs && !psEncC->first_frame_after_reset && psEncC->nb_subfr == MAX_NB_SUBFR));
}
//----------------------------------------------------------------------------------------------------------------------

void silk_find_LTP_FIX(int32_t XXLTP_Q17[MAX_NB_SUBFR * LTP_ORDER * LTP_ORDER], /* O    Correlation matrix           */
                       int32_t xXLTP_Q17[MAX_NB_SUBFR * LTP_ORDER],             /* O    Correlation vector           */
                       const int16_t r_ptr[],                                   /* I    Residual signal after LPC    */
                       const int32_t lag[MAX_NB_SUBFR],                         /* I    LTP lags                     */
                       const int32_t subfr_length,                              /* I    Subframe length              */
                       const int32_t nb_subfr,                                  /* I    Number of subframes          */
                       int arch                                                 /* I    Run-time architecture        */
) {
    int32_t i, k, extra_shifts;
    int32_t xx_shifts, xX_shifts, XX_shifts;
    const int16_t *lag_ptr;
    int32_t *XXLTP_Q17_ptr, *xXLTP_Q17_ptr;
    int32_t xx, nrg, temp;

    xXLTP_Q17_ptr = xXLTP_Q17;
    XXLTP_Q17_ptr = XXLTP_Q17;
    for (k = 0; k < nb_subfr; k++) {
        lag_ptr = r_ptr - (lag[k] + LTP_ORDER / 2);

        silk_sum_sqr_shift(&xx, &xx_shifts, r_ptr, subfr_length + LTP_ORDER); /* xx in Q( -xx_shifts ) */
        silk_corrMatrix_FIX(lag_ptr, subfr_length, LTP_ORDER, XXLTP_Q17_ptr, &nrg, &XX_shifts,
                            arch); /* XXLTP_Q17_ptr and nrg in Q( -XX_shifts ) */
        extra_shifts = xx_shifts - XX_shifts;
        if (extra_shifts > 0) {
            /* Shift XX */
            xX_shifts = xx_shifts;
            for (i = 0; i < LTP_ORDER * LTP_ORDER; i++) {
                XXLTP_Q17_ptr[i] = silk_RSHIFT32(XXLTP_Q17_ptr[i], extra_shifts); /* Q( -xX_shifts ) */
            }
            nrg = silk_RSHIFT32(nrg, extra_shifts); /* Q( -xX_shifts ) */
        } else if (extra_shifts < 0) {
            /* Shift xx */
            xX_shifts = XX_shifts;
            xx = silk_RSHIFT32(xx, -extra_shifts); /* Q( -xX_shifts ) */
        } else {
            xX_shifts = xx_shifts;
        }
        silk_corrVector_FIX(lag_ptr, r_ptr, subfr_length, LTP_ORDER, xXLTP_Q17_ptr, xX_shifts,
                            arch); /* xXLTP_Q17_ptr in Q( -xX_shifts ) */

        /* At this point all correlations are in Q(-xX_shifts) */
        temp = silk_SMLAWB(1, nrg, SILK_FIX_CONST(LTP_CORR_INV_MAX, 16));
        temp = silk_max(temp, xx);
        TIC(div)

        for (i = 0; i < LTP_ORDER * LTP_ORDER; i++) {
            XXLTP_Q17_ptr[i] = (int32_t)(silk_LSHIFT64((int64_t)XXLTP_Q17_ptr[i], 17) / temp);
        }
        for (i = 0; i < LTP_ORDER; i++) {
            xXLTP_Q17_ptr[i] = (int32_t)(silk_LSHIFT64((int64_t)xXLTP_Q17_ptr[i], 17) / temp);
        }

        TOC(div)
        r_ptr += subfr_length;
        XXLTP_Q17_ptr += LTP_ORDER * LTP_ORDER;
        xXLTP_Q17_ptr += LTP_ORDER;
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Find pitch lags */
void silk_find_pitch_lags_FIX(silk_encoder_state_FIX *psEnc,       /* I/O  encoder state       */
                              silk_encoder_control_FIX *psEncCtrl, /* I/O  encoder control */
                              int16_t res[],                       /* O    residual                       */
                              const int16_t x[],                   /* I    Speech signal                   */
                              int arch                             /* I    Run-time architecture                             */
) {
    int32_t buf_len, i, scale;
    int32_t thrhld_Q13, res_nrg;
    const int16_t *x_ptr;
    VARDECL(int16_t, Wsig);
    int16_t *Wsig_ptr;
    int32_t auto_corr[MAX_FIND_PITCH_LPC_ORDER + 1];
    int16_t rc_Q15[MAX_FIND_PITCH_LPC_ORDER];
    int32_t A_Q24[MAX_FIND_PITCH_LPC_ORDER];
    int16_t A_Q12[MAX_FIND_PITCH_LPC_ORDER];
    SAVE_STACK;

    /******************************************/
    /* Set up buffer lengths etc based on Fs  */
    /******************************************/
    buf_len = psEnc->sCmn.la_pitch + psEnc->sCmn.frame_length + psEnc->sCmn.ltp_mem_length;

    /* Safety check */
    assert(buf_len >= psEnc->sCmn.pitch_LPC_win_length);

    /*************************************/
    /* Estimate LPC AR coefficients      */
    /*************************************/

    /* Calculate windowed signal */

    ALLOC(Wsig, psEnc->sCmn.pitch_LPC_win_length, int16_t);

    /* First LA_LTP samples */
    x_ptr = x + buf_len - psEnc->sCmn.pitch_LPC_win_length;
    Wsig_ptr = Wsig;
    silk_apply_sine_window(Wsig_ptr, x_ptr, 1, psEnc->sCmn.la_pitch);

    /* Middle un - windowed samples */
    Wsig_ptr += psEnc->sCmn.la_pitch;
    x_ptr += psEnc->sCmn.la_pitch;
    silk_memcpy(Wsig_ptr, x_ptr,
                (psEnc->sCmn.pitch_LPC_win_length - silk_LSHIFT(psEnc->sCmn.la_pitch, 1)) * sizeof(int16_t));

    /* Last LA_LTP samples */
    Wsig_ptr += psEnc->sCmn.pitch_LPC_win_length - silk_LSHIFT(psEnc->sCmn.la_pitch, 1);
    x_ptr += psEnc->sCmn.pitch_LPC_win_length - silk_LSHIFT(psEnc->sCmn.la_pitch, 1);
    silk_apply_sine_window(Wsig_ptr, x_ptr, 2, psEnc->sCmn.la_pitch);

    /* Calculate autocorrelation sequence */
    silk_autocorr(auto_corr, &scale, Wsig, psEnc->sCmn.pitch_LPC_win_length, psEnc->sCmn.pitchEstimationLPCOrder + 1,
                  arch);

    /* Add white noise, as fraction of energy */
    auto_corr[0] = silk_SMLAWB(auto_corr[0], auto_corr[0], SILK_FIX_CONST(FIND_PITCH_WHITE_NOISE_FRACTION, 16)) + 1;

    /* Calculate the reflection coefficients using schur */
    res_nrg = silk_schur(rc_Q15, auto_corr, psEnc->sCmn.pitchEstimationLPCOrder);

    /* Prediction gain */
    psEncCtrl->predGain_Q16 = silk_DIV32_varQ(auto_corr[0], silk_max_int(res_nrg, 1), 16);

    /* Convert reflection coefficients to prediction coefficients */
    silk_k2a(A_Q24, rc_Q15, psEnc->sCmn.pitchEstimationLPCOrder);

    /* Convert From 32 bit Q24 to 16 bit Q12 coefs */
    for (i = 0; i < psEnc->sCmn.pitchEstimationLPCOrder; i++) {
        A_Q12[i] = (int16_t)silk_SAT16(silk_RSHIFT(A_Q24[i], 12));
    }

    /* Do BWE */
    silk_bwexpander(A_Q12, psEnc->sCmn.pitchEstimationLPCOrder, SILK_FIX_CONST(FIND_PITCH_BANDWIDTH_EXPANSION, 16));

    /*****************************************/
    /* LPC analysis filtering                */
    /*****************************************/
    silk_LPC_analysis_filter(res, x, A_Q12, buf_len, psEnc->sCmn.pitchEstimationLPCOrder, psEnc->sCmn.arch);

    if (psEnc->sCmn.indices.signalType != TYPE_NO_VOICE_ACTIVITY && psEnc->sCmn.first_frame_after_reset == 0) {
        /* Threshold for pitch estimator */
        thrhld_Q13 = SILK_FIX_CONST(0.6, 13);
        thrhld_Q13 = silk_SMLABB(thrhld_Q13, SILK_FIX_CONST(-0.004, 13), psEnc->sCmn.pitchEstimationLPCOrder);
        thrhld_Q13 = silk_SMLAWB(thrhld_Q13, SILK_FIX_CONST(-0.1, 21), psEnc->sCmn.speech_activity_Q8);
        thrhld_Q13 = silk_SMLABB(thrhld_Q13, SILK_FIX_CONST(-0.15, 13), silk_RSHIFT(psEnc->sCmn.prevSignalType, 1));
        thrhld_Q13 = silk_SMLAWB(thrhld_Q13, SILK_FIX_CONST(-0.1, 14), psEnc->sCmn.input_tilt_Q15);
        thrhld_Q13 = silk_SAT16(thrhld_Q13);

        /*****************************************/
        /* Call pitch estimator                  */
        /*****************************************/
        if (silk_pitch_analysis_core(res, psEncCtrl->pitchL, &psEnc->sCmn.indices.lagIndex,
                                     &psEnc->sCmn.indices.contourIndex, &psEnc->LTPCorr_Q15, psEnc->sCmn.prevLag,
                                     psEnc->sCmn.pitchEstimationThreshold_Q16, (int32_t)thrhld_Q13, psEnc->sCmn.fs_kHz,
                                     psEnc->sCmn.pitchEstimationComplexity, psEnc->sCmn.nb_subfr,
                                     psEnc->sCmn.arch) == 0) {
            psEnc->sCmn.indices.signalType = TYPE_VOICED;
        } else {
            psEnc->sCmn.indices.signalType = TYPE_UNVOICED;
        }
    } else {
        silk_memset(psEncCtrl->pitchL, 0, sizeof(psEncCtrl->pitchL));
        psEnc->sCmn.indices.lagIndex = 0;
        psEnc->sCmn.indices.contourIndex = 0;
        psEnc->LTPCorr_Q15 = 0;
    }
}
//----------------------------------------------------------------------------------------------------------------------

void silk_find_pred_coefs_FIX(silk_encoder_state_FIX *psEnc,       /* I/O  encoder state                          */
                              silk_encoder_control_FIX *psEncCtrl, /* I/O  encoder control                        */
                              const int16_t res_pitch[],           /* I    Residual from pitch analysis           */
                              const int16_t x[],                   /* I    Speech signal                          */
                              int32_t condCoding                   /* I    The type of conditional coding to use  */
) {
    int32_t i;
    int32_t invGains_Q16[MAX_NB_SUBFR], local_gains[MAX_NB_SUBFR];
    int16_t NLSF_Q15[MAX_LPC_ORDER];
    const int16_t *x_ptr;
    int16_t *x_pre_ptr;
    VARDECL(int16_t, LPC_in_pre);
    int32_t min_gain_Q16, minInvGain_Q30;
    SAVE_STACK;

    /* weighting for weighted least squares */
    min_gain_Q16 = silk_int32_MAX >> 6;
    for (i = 0; i < psEnc->sCmn.nb_subfr; i++) {
        min_gain_Q16 = silk_min(min_gain_Q16, psEncCtrl->Gains_Q16[i]);
    }
    for (i = 0; i < psEnc->sCmn.nb_subfr; i++) {
        /* Divide to Q16 */
        assert(psEncCtrl->Gains_Q16[i] > 0);
        /* Invert and normalize gains, and ensure that maximum invGains_Q16 is within range of a 16 bit int */
        invGains_Q16[i] = silk_DIV32_varQ(min_gain_Q16, psEncCtrl->Gains_Q16[i], 16 - 2);

        /* Limit inverse */
        invGains_Q16[i] = silk_max(invGains_Q16[i], 100);

        /* Square the inverted gains */
        assert(invGains_Q16[i] == silk_SAT16(invGains_Q16[i]));

        /* Invert the inverted and normalized gains */
        local_gains[i] = silk_DIV32(((int32_t)1 << 16), invGains_Q16[i]);
    }

    ALLOC(LPC_in_pre, psEnc->sCmn.nb_subfr * psEnc->sCmn.predictLPCOrder + psEnc->sCmn.frame_length, int16_t);
    if (psEnc->sCmn.indices.signalType == TYPE_VOICED) {
        VARDECL(int32_t, xXLTP_Q17);
        VARDECL(int32_t, XXLTP_Q17);

        /**********/
        /* VOICED */
        /**********/
        assert(psEnc->sCmn.ltp_mem_length - psEnc->sCmn.predictLPCOrder >= psEncCtrl->pitchL[0] + LTP_ORDER / 2);

        ALLOC(xXLTP_Q17, psEnc->sCmn.nb_subfr * LTP_ORDER, int32_t);
        ALLOC(XXLTP_Q17, psEnc->sCmn.nb_subfr * LTP_ORDER * LTP_ORDER, int32_t);

        /* LTP analysis */
        silk_find_LTP_FIX(XXLTP_Q17, xXLTP_Q17, res_pitch, psEncCtrl->pitchL, psEnc->sCmn.subfr_length,
                          psEnc->sCmn.nb_subfr, psEnc->sCmn.arch);

        /* Quantize LTP gain parameters */
        silk_quant_LTP_gains(psEncCtrl->LTPCoef_Q14, psEnc->sCmn.indices.LTPIndex, &psEnc->sCmn.indices.PERIndex,
                             &psEnc->sCmn.sum_log_gain_Q7, &psEncCtrl->LTPredCodGain_Q7, XXLTP_Q17, xXLTP_Q17,
                             psEnc->sCmn.subfr_length, psEnc->sCmn.nb_subfr, psEnc->sCmn.arch);

        /* Control LTP scaling */
        silk_LTP_scale_ctrl_FIX(psEnc, psEncCtrl, condCoding);

        /* Create LTP residual */
        silk_LTP_analysis_filter_FIX(LPC_in_pre, x - psEnc->sCmn.predictLPCOrder, psEncCtrl->LTPCoef_Q14,
                                     psEncCtrl->pitchL, invGains_Q16, psEnc->sCmn.subfr_length, psEnc->sCmn.nb_subfr,
                                     psEnc->sCmn.predictLPCOrder);

    } else {
        /************/
        /* UNVOICED */
        /************/
        /* Create signal with prepended subframes, scaled by inverse gains */
        x_ptr = x - psEnc->sCmn.predictLPCOrder;
        x_pre_ptr = LPC_in_pre;
        for (i = 0; i < psEnc->sCmn.nb_subfr; i++) {
            silk_scale_copy_vector16(x_pre_ptr, x_ptr, invGains_Q16[i],
                                     psEnc->sCmn.subfr_length + psEnc->sCmn.predictLPCOrder);
            x_pre_ptr += psEnc->sCmn.subfr_length + psEnc->sCmn.predictLPCOrder;
            x_ptr += psEnc->sCmn.subfr_length;
        }

        silk_memset(psEncCtrl->LTPCoef_Q14, 0, psEnc->sCmn.nb_subfr * LTP_ORDER * sizeof(int16_t));
        psEncCtrl->LTPredCodGain_Q7 = 0;
        psEnc->sCmn.sum_log_gain_Q7 = 0;
    }

    /* Limit on total predictive coding gain */
    if (psEnc->sCmn.first_frame_after_reset) {
        minInvGain_Q30 = SILK_FIX_CONST(1.0f / MAX_PREDICTION_POWER_GAIN_AFTER_RESET, 30);
    } else {
        minInvGain_Q30 = silk_log2lin(
            silk_SMLAWB(16 << 7, (int32_t)psEncCtrl->LTPredCodGain_Q7, SILK_FIX_CONST(1.0 / 3, 16))); /* Q16 */
        minInvGain_Q30 = silk_DIV32_varQ(
            minInvGain_Q30,
            silk_SMULWW(SILK_FIX_CONST(MAX_PREDICTION_POWER_GAIN, 0),
                        silk_SMLAWB(SILK_FIX_CONST(0.25, 18), SILK_FIX_CONST(0.75, 18), psEncCtrl->coding_quality_Q14)),
            14);
    }

    /* LPC_in_pre contains the LTP-filtered input for voiced, and the unfiltered input for unvoiced */
    silk_find_LPC_FIX(&psEnc->sCmn, NLSF_Q15, LPC_in_pre, minInvGain_Q30);

    /* Quantize LSFs */
    silk_process_NLSFs(&psEnc->sCmn, psEncCtrl->PredCoef_Q12, NLSF_Q15, psEnc->sCmn.prev_NLSFq_Q15);

    /* Calculate residual energy using quantized LPC coefficients */
    silk_residual_energy_FIX(psEncCtrl->ResNrg, psEncCtrl->ResNrgQ, LPC_in_pre, psEncCtrl->PredCoef_Q12, local_gains,
                             psEnc->sCmn.subfr_length, psEnc->sCmn.nb_subfr, psEnc->sCmn.predictLPCOrder,
                             psEnc->sCmn.arch);

    /* Copy to prediction struct for use in next frame for interpolation */
    silk_memcpy(psEnc->sCmn.prev_NLSFq_Q15, NLSF_Q15, sizeof(psEnc->sCmn.prev_NLSFq_Q15));
}
//----------------------------------------------------------------------------------------------------------------------

/* Step up function, converts reflection coefficients to prediction coefficients                         */
void silk_k2a(int32_t *A_Q24,        /* O    Prediction coefficients [order] Q24                         */
              const int16_t *rc_Q15, /* I    Reflection coefficients [order] Q15                         */
              const int32_t order    /* I    Prediction order                                            */
) {
    int32_t k, n;
    int32_t rc, tmp1, tmp2;

    for (k = 0; k < order; k++) {
        rc = rc_Q15[k];
        for (n = 0; n < (k + 1) >> 1; n++) {
            tmp1 = A_Q24[n];
            tmp2 = A_Q24[k - n - 1];
            A_Q24[n] = silk_SMLAWB(tmp1, silk_LSHIFT(tmp2, 1), rc);
            A_Q24[k - n - 1] = silk_SMLAWB(tmp2, silk_LSHIFT(tmp1, 1), rc);
        }
        A_Q24[k] = -silk_LSHIFT((int32_t)rc_Q15[k], 9);
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Step up function, converts reflection coefficients to prediction coefficients */
void silk_k2a_Q16(int32_t *A_Q24,        /* O    Prediction coefficients [order] Q24                         */
                  const int32_t *rc_Q16, /* I    Reflection coefficients [order] Q16                         */
                  const int32_t order    /* I    Prediction order                                            */
) {
    int32_t k, n;
    int32_t rc, tmp1, tmp2;

    for (k = 0; k < order; k++) {
        rc = rc_Q16[k];
        for (n = 0; n < (k + 1) >> 1; n++) {
            tmp1 = A_Q24[n];
            tmp2 = A_Q24[k - n - 1];
            A_Q24[n] = silk_SMLAWW(tmp1, tmp2, rc);
            A_Q24[k - n - 1] = silk_SMLAWW(tmp2, tmp1, rc);
        }
        A_Q24[k] = -silk_LSHIFT(rc, 8);
    }
}
//----------------------------------------------------------------------------------------------------------------------

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
) {
    const int16_t *x_ptr, *x_lag_ptr;
    int16_t Btmp_Q14[LTP_ORDER];
    int16_t *LTP_res_ptr;
    int32_t k, i;
    int32_t LTP_est;

    x_ptr = x;
    LTP_res_ptr = LTP_res;
    for (k = 0; k < nb_subfr; k++) {
        x_lag_ptr = x_ptr - pitchL[k];

        Btmp_Q14[0] = LTPCoef_Q14[k * LTP_ORDER];
        Btmp_Q14[1] = LTPCoef_Q14[k * LTP_ORDER + 1];
        Btmp_Q14[2] = LTPCoef_Q14[k * LTP_ORDER + 2];
        Btmp_Q14[3] = LTPCoef_Q14[k * LTP_ORDER + 3];
        Btmp_Q14[4] = LTPCoef_Q14[k * LTP_ORDER + 4];

        /* LTP analysis FIR filter */
        for (i = 0; i < subfr_length + pre_length; i++) {
            LTP_res_ptr[i] = x_ptr[i];

            /* Long-term prediction */
            LTP_est = silk_SMULBB(x_lag_ptr[LTP_ORDER / 2], Btmp_Q14[0]);
            LTP_est = silk_SMLABB_ovflw(LTP_est, x_lag_ptr[1], Btmp_Q14[1]);
            LTP_est = silk_SMLABB_ovflw(LTP_est, x_lag_ptr[0], Btmp_Q14[2]);
            LTP_est = silk_SMLABB_ovflw(LTP_est, x_lag_ptr[-1], Btmp_Q14[3]);
            LTP_est = silk_SMLABB_ovflw(LTP_est, x_lag_ptr[-2], Btmp_Q14[4]);

            LTP_est = silk_RSHIFT_ROUND(LTP_est, 14); /* round and -> Q0*/

            /* Subtract long-term prediction */
            LTP_res_ptr[i] = (int16_t)silk_SAT16((int32_t)x_ptr[i] - LTP_est);

            /* Scale residual */
            LTP_res_ptr[i] = silk_SMULWB(invGains_Q16[k], LTP_res_ptr[i]);

            x_lag_ptr++;
        }

        /* Update pointers */
        LTP_res_ptr += subfr_length + pre_length;
        x_ptr += subfr_length;
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Calculation of LTP state scaling */
void silk_LTP_scale_ctrl_FIX(silk_encoder_state_FIX *psEnc,       /* I/O  encoder state                          */
                             silk_encoder_control_FIX *psEncCtrl, /* I/O  encoder control                        */
                             int32_t condCoding                   /* I    The type of conditional coding to use  */
) {
    int32_t round_loss;

    if (condCoding == CODE_INDEPENDENTLY) {
        /* Only scale if first frame in packet */
        round_loss = psEnc->sCmn.PacketLoss_perc + psEnc->sCmn.nFramesPerPacket;
        psEnc->sCmn.indices.LTP_scaleIndex = (int8_t)silk_LIMIT(
            silk_SMULWB(silk_SMULBB(round_loss, psEncCtrl->LTPredCodGain_Q7), SILK_FIX_CONST(0.1, 9)), 0, 2);
    } else {
        /* Default is minimum scaling */
        psEnc->sCmn.indices.LTP_scaleIndex = 0;
    }
    psEncCtrl->LTP_scale_Q14 = silk_LTPScales_table_Q14[psEnc->sCmn.indices.LTP_scaleIndex];
}
//----------------------------------------------------------------------------------------------------------------------

/* Compute gain to make warped filter coefficients have a zero mean log frequency response on a   */
/* non-warped frequency scale. (So that it can be implemented with a minimum-phase monic filter.) */
/* Note: A monic filter is one with the first coefficient equal to 1.0. In Silk we omit the first */
/* coefficient in an array of coefficients, for monic filters.                                    */
static inline int32_t warped_gain(/* gain in Q16*/
                                       const int32_t *coefs_Q24, int32_t lambda_Q16, int32_t order) {
    int32_t i;
    int32_t gain_Q24;

    lambda_Q16 = -lambda_Q16;
    gain_Q24 = coefs_Q24[order - 1];
    for (i = order - 2; i >= 0; i--) {
        gain_Q24 = silk_SMLAWB(coefs_Q24[i], gain_Q24, lambda_Q16);
    }
    gain_Q24 = silk_SMLAWB(SILK_FIX_CONST(1.0, 24), gain_Q24, -lambda_Q16);
    return silk_INVERSE32_varQ(gain_Q24, 40);
}
//----------------------------------------------------------------------------------------------------------------------

/* Convert warped filter coefficients to monic pseudo-warped coefficients and limit maximum     */
/* amplitude of monic warped coefficients by using bandwidth expansion on the true coefficients */
static inline void limit_warped_coefs(int32_t *coefs_Q24, int32_t lambda_Q16, int32_t limit_Q24, int32_t order) {
    int32_t i, iter, ind = 0;
    int32_t tmp, maxabs_Q24, chirp_Q16, gain_Q16;
    int32_t nom_Q16, den_Q24;
    int32_t limit_Q20, maxabs_Q20;

    /* Convert to monic coefficients */
    lambda_Q16 = -lambda_Q16;
    for (i = order - 1; i > 0; i--) {
        coefs_Q24[i - 1] = silk_SMLAWB(coefs_Q24[i - 1], coefs_Q24[i], lambda_Q16);
    }
    lambda_Q16 = -lambda_Q16;
    nom_Q16 = silk_SMLAWB(SILK_FIX_CONST(1.0, 16), -(int32_t)lambda_Q16, lambda_Q16);
    den_Q24 = silk_SMLAWB(SILK_FIX_CONST(1.0, 24), coefs_Q24[0], lambda_Q16);
    gain_Q16 = silk_DIV32_varQ(nom_Q16, den_Q24, 24);
    for (i = 0; i < order; i++) {
        coefs_Q24[i] = silk_SMULWW(gain_Q16, coefs_Q24[i]);
    }
    limit_Q20 = silk_RSHIFT(limit_Q24, 4);
    for (iter = 0; iter < 10; iter++) {
        /* Find maximum absolute value */
        maxabs_Q24 = -1;
        for (i = 0; i < order; i++) {
            tmp = silk_abs_int32(coefs_Q24[i]);
            if (tmp > maxabs_Q24) {
                maxabs_Q24 = tmp;
                ind = i;
            }
        }
        /* Use Q20 to avoid any overflow when multiplying by (ind + 1) later. */
        maxabs_Q20 = silk_RSHIFT(maxabs_Q24, 4);
        if (maxabs_Q20 <= limit_Q20) {
            /* Coefficients are within range - done */
            return;
        }

        /* Convert back to true warped coefficients */
        for (i = 1; i < order; i++) {
            coefs_Q24[i - 1] = silk_SMLAWB(coefs_Q24[i - 1], coefs_Q24[i], lambda_Q16);
        }
        gain_Q16 = silk_INVERSE32_varQ(gain_Q16, 32);
        for (i = 0; i < order; i++) {
            coefs_Q24[i] = silk_SMULWW(gain_Q16, coefs_Q24[i]);
        }

        /* Apply bandwidth expansion */
        chirp_Q16 = SILK_FIX_CONST(0.99, 16) -
                    silk_DIV32_varQ(silk_SMULWB(maxabs_Q20 - limit_Q20,
                                                silk_SMLABB(SILK_FIX_CONST(0.8, 10), SILK_FIX_CONST(0.1, 10), iter)),
                                    silk_MUL(maxabs_Q20, ind + 1), 22);
        silk_bwexpander_32(coefs_Q24, order, chirp_Q16);

        /* Convert to monic warped coefficients */
        lambda_Q16 = -lambda_Q16;
        for (i = order - 1; i > 0; i--) {
            coefs_Q24[i - 1] = silk_SMLAWB(coefs_Q24[i - 1], coefs_Q24[i], lambda_Q16);
        }
        lambda_Q16 = -lambda_Q16;
        nom_Q16 = silk_SMLAWB(SILK_FIX_CONST(1.0, 16), -(int32_t)lambda_Q16, lambda_Q16);
        den_Q24 = silk_SMLAWB(SILK_FIX_CONST(1.0, 24), coefs_Q24[0], lambda_Q16);
        gain_Q16 = silk_DIV32_varQ(nom_Q16, den_Q24, 24);
        for (i = 0; i < order; i++) {
            coefs_Q24[i] = silk_SMULWW(gain_Q16, coefs_Q24[i]);
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Disable MIPS version until it's updated. */

/**************************************************************/
/* Compute noise shaping coefficients and initial gain values */
/**************************************************************/
void silk_noise_shape_analysis_FIX(silk_encoder_state_FIX *psEnc,       /* I/O  Encoder state FIX       */
                                   silk_encoder_control_FIX *psEncCtrl, /* I/O  Encoder control FIX */
                                   const int16_t *pitch_res,            /* I    LPC residual from pitch analysis            */
                                   const int16_t *x, /* I    Input signal [ frame_length + la_shape ] */
                                   int arch          /* I    Run-time architecture          */
) {
    silk_shape_state_FIX *psShapeSt = &psEnc->sShape;
    int32_t k, i, nSamples, nSegs, Qnrg, b_Q14, warping_Q16, scale = 0;
    int32_t SNR_adj_dB_Q7, HarmShapeGain_Q16, Tilt_Q16, tmp32;
    int32_t nrg, log_energy_Q7, log_energy_prev_Q7, energy_variation_Q7;
    int32_t BWExp_Q16, gain_mult_Q16, gain_add_Q16, strength_Q16, b_Q8;
    int32_t auto_corr[MAX_SHAPE_LPC_ORDER + 1];
    int32_t refl_coef_Q16[MAX_SHAPE_LPC_ORDER];
    int32_t AR_Q24[MAX_SHAPE_LPC_ORDER];
    VARDECL(int16_t, x_windowed);
    const int16_t *x_ptr, *pitch_res_ptr;
    SAVE_STACK;

    /* Point to start of first LPC analysis block */
    x_ptr = x - psEnc->sCmn.la_shape;

    /****************/
    /* GAIN CONTROL */
    /****************/
    SNR_adj_dB_Q7 = psEnc->sCmn.SNR_dB_Q7;

    /* Input quality is the average of the quality in the lowest two VAD bands */
    psEncCtrl->input_quality_Q14 = (int32_t)silk_RSHIFT(
        (int32_t)psEnc->sCmn.input_quality_bands_Q15[0] + psEnc->sCmn.input_quality_bands_Q15[1], 2);

    /* Coding quality level, between 0.0_Q0 and 1.0_Q0, but in Q14 */
    psEncCtrl->coding_quality_Q14 =
        silk_RSHIFT(silk_sigm_Q15(silk_RSHIFT_ROUND(SNR_adj_dB_Q7 - SILK_FIX_CONST(20.0, 7), 4)), 1);

    /* Reduce coding SNR during low speech activity */
    if (psEnc->sCmn.useCBR == 0) {
        b_Q8 = SILK_FIX_CONST(1.0, 8) - psEnc->sCmn.speech_activity_Q8;
        b_Q8 = silk_SMULWB(silk_LSHIFT(b_Q8, 8), b_Q8);
        SNR_adj_dB_Q7 =
            silk_SMLAWB(SNR_adj_dB_Q7, silk_SMULBB(SILK_FIX_CONST(-BG_SNR_DECR_dB, 7) >> (4 + 1), b_Q8), /* Q11*/
                        silk_SMULWB(SILK_FIX_CONST(1.0, 14) + psEncCtrl->input_quality_Q14,
                                    psEncCtrl->coding_quality_Q14)); /* Q12*/
    }

    if (psEnc->sCmn.indices.signalType == TYPE_VOICED) {
        /* Reduce gains for periodic signals */
        SNR_adj_dB_Q7 = silk_SMLAWB(SNR_adj_dB_Q7, SILK_FIX_CONST(HARM_SNR_INCR_dB, 8), psEnc->LTPCorr_Q15);
    } else {
        /* For unvoiced signals and low-quality input, adjust the quality slower than SNR_dB setting */
        SNR_adj_dB_Q7 = silk_SMLAWB(
            SNR_adj_dB_Q7, silk_SMLAWB(SILK_FIX_CONST(6.0, 9), -SILK_FIX_CONST(0.4, 18), psEnc->sCmn.SNR_dB_Q7),
            SILK_FIX_CONST(1.0, 14) - psEncCtrl->input_quality_Q14);
    }

    /*************************/
    /* SPARSENESS PROCESSING */
    /*************************/
    /* Set quantizer offset */
    if (psEnc->sCmn.indices.signalType == TYPE_VOICED) {
        /* Initially set to 0; may be overruled in process_gains(..) */
        psEnc->sCmn.indices.quantOffsetType = 0;
    } else {
        /* Sparseness measure, based on relative fluctuations of energy per 2 milliseconds */
        nSamples = silk_LSHIFT(psEnc->sCmn.fs_kHz, 1);
        energy_variation_Q7 = 0;
        log_energy_prev_Q7 = 0;
        pitch_res_ptr = pitch_res;
        nSegs = silk_SMULBB(SUB_FRAME_LENGTH_MS, psEnc->sCmn.nb_subfr) / 2;
        for (k = 0; k < nSegs; k++) {
            silk_sum_sqr_shift(&nrg, &scale, pitch_res_ptr, nSamples);
            nrg += silk_RSHIFT(nSamples, scale); /* Q(-scale)*/

            log_energy_Q7 = silk_lin2log(nrg);
            if (k > 0) {
                energy_variation_Q7 += silk_abs(log_energy_Q7 - log_energy_prev_Q7);
            }
            log_energy_prev_Q7 = log_energy_Q7;
            pitch_res_ptr += nSamples;
        }

        /* Set quantization offset depending on sparseness measure */
        if (energy_variation_Q7 > SILK_FIX_CONST(ENERGY_VARIATION_THRESHOLD_QNT_OFFSET, 7) * (nSegs - 1)) {
            psEnc->sCmn.indices.quantOffsetType = 0;
        } else {
            psEnc->sCmn.indices.quantOffsetType = 1;
        }
    }

    /*******************************/
    /* Control bandwidth expansion */
    /*******************************/
    /* More BWE for signals with high prediction gain */
    strength_Q16 = silk_SMULWB(psEncCtrl->predGain_Q16, SILK_FIX_CONST(FIND_PITCH_WHITE_NOISE_FRACTION, 16));
    BWExp_Q16 = silk_DIV32_varQ(SILK_FIX_CONST(BANDWIDTH_EXPANSION, 16),
                                silk_SMLAWW(SILK_FIX_CONST(1.0, 16), strength_Q16, strength_Q16), 16);

    if (psEnc->sCmn.warping_Q16 > 0) {
        /* Slightly more warping in analysis will move quantization noise up in frequency, where it's better masked */
        warping_Q16 =
            silk_SMLAWB(psEnc->sCmn.warping_Q16, (int32_t)psEncCtrl->coding_quality_Q14, SILK_FIX_CONST(0.01, 18));
    } else {
        warping_Q16 = 0;
    }

    /********************************************/
    /* Compute noise shaping AR coefs and gains */
    /********************************************/
    ALLOC(x_windowed, psEnc->sCmn.shapeWinLength, int16_t);
    for (k = 0; k < psEnc->sCmn.nb_subfr; k++) {
        /* Apply window: sine slope followed by flat part followed by cosine slope */
        int32_t shift, slope_part, flat_part;
        flat_part = psEnc->sCmn.fs_kHz * 3;
        slope_part = silk_RSHIFT(psEnc->sCmn.shapeWinLength - flat_part, 1);

        silk_apply_sine_window(x_windowed, x_ptr, 1, slope_part);
        shift = slope_part;
        silk_memcpy(x_windowed + shift, x_ptr + shift, flat_part * sizeof(int16_t));
        shift += flat_part;
        silk_apply_sine_window(x_windowed + shift, x_ptr + shift, 2, slope_part);

        /* Update pointer: next LPC analysis block */
        x_ptr += psEnc->sCmn.subfr_length;

        if (psEnc->sCmn.warping_Q16 > 0) {
            /* Calculate warped auto correlation */
            silk_warped_autocorrelation_FIX(auto_corr, &scale, x_windowed, warping_Q16, psEnc->sCmn.shapeWinLength,
                                            psEnc->sCmn.shapingLPCOrder, arch);
        } else {
            /* Calculate regular auto correlation */
            silk_autocorr(auto_corr, &scale, x_windowed, psEnc->sCmn.shapeWinLength, psEnc->sCmn.shapingLPCOrder + 1,
                          arch);
        }

        /* Add white noise, as a fraction of energy */
        auto_corr[0] = silk_ADD32(
            auto_corr[0],
            silk_max_32(silk_SMULWB(silk_RSHIFT(auto_corr[0], 4), SILK_FIX_CONST(SHAPE_WHITE_NOISE_FRACTION, 20)), 1));

        /* Calculate the reflection coefficients using schur */
        nrg = silk_schur64(refl_coef_Q16, auto_corr, psEnc->sCmn.shapingLPCOrder);
        assert(nrg >= 0);

        /* Convert reflection coefficients to prediction coefficients */
        silk_k2a_Q16(AR_Q24, refl_coef_Q16, psEnc->sCmn.shapingLPCOrder);

        Qnrg = -scale; /* range: -12...30*/
        assert(Qnrg >= -12);
        assert(Qnrg <= 30);

        /* Make sure that Qnrg is an even number */
        if (Qnrg & 1) {
            Qnrg -= 1;
            nrg >>= 1;
        }

        tmp32 = silk_SQRT_APPROX(nrg);
        Qnrg >>= 1; /* range: -6...15*/

        psEncCtrl->Gains_Q16[k] = silk_LSHIFT_SAT32(tmp32, 16 - Qnrg);

        if (psEnc->sCmn.warping_Q16 > 0) {
            /* Adjust gain for warping */
            gain_mult_Q16 = warped_gain(AR_Q24, warping_Q16, psEnc->sCmn.shapingLPCOrder);
            assert(psEncCtrl->Gains_Q16[k] > 0);
            if (psEncCtrl->Gains_Q16[k] < SILK_FIX_CONST(0.25, 16)) {
                psEncCtrl->Gains_Q16[k] = silk_SMULWW(psEncCtrl->Gains_Q16[k], gain_mult_Q16);
            } else {
                psEncCtrl->Gains_Q16[k] = silk_SMULWW(silk_RSHIFT_ROUND(psEncCtrl->Gains_Q16[k], 1), gain_mult_Q16);
                if (psEncCtrl->Gains_Q16[k] >= (silk_int32_MAX >> 1)) {
                    psEncCtrl->Gains_Q16[k] = silk_int32_MAX;
                } else {
                    psEncCtrl->Gains_Q16[k] = silk_LSHIFT32(psEncCtrl->Gains_Q16[k], 1);
                }
            }
            assert(psEncCtrl->Gains_Q16[k] > 0);
        }

        /* Bandwidth expansion */
        silk_bwexpander_32(AR_Q24, psEnc->sCmn.shapingLPCOrder, BWExp_Q16);

        if (psEnc->sCmn.warping_Q16 > 0) {
            /* Convert to monic warped prediction coefficients and limit absolute values */
            limit_warped_coefs(AR_Q24, warping_Q16, SILK_FIX_CONST(3.999, 24), psEnc->sCmn.shapingLPCOrder);

            /* Convert from Q24 to Q13 and store in int16 */
            for (i = 0; i < psEnc->sCmn.shapingLPCOrder; i++) {
                psEncCtrl->AR_Q13[k * MAX_SHAPE_LPC_ORDER + i] = (int16_t)silk_SAT16(silk_RSHIFT_ROUND(AR_Q24[i], 11));
            }
        } else {
            silk_LPC_fit(&psEncCtrl->AR_Q13[k * MAX_SHAPE_LPC_ORDER], AR_Q24, 13, 24, psEnc->sCmn.shapingLPCOrder);
        }
    }

    /*****************/
    /* Gain tweaking */
    /*****************/
    /* Increase gains during low speech activity and put lower limit on gains */
    gain_mult_Q16 = silk_log2lin(-silk_SMLAWB(-SILK_FIX_CONST(16.0, 7), SNR_adj_dB_Q7, SILK_FIX_CONST(0.16, 16)));
    gain_add_Q16 =
        silk_log2lin(silk_SMLAWB(SILK_FIX_CONST(16.0, 7), SILK_FIX_CONST(MIN_QGAIN_DB, 7), SILK_FIX_CONST(0.16, 16)));
    assert(gain_mult_Q16 > 0);
    for (k = 0; k < psEnc->sCmn.nb_subfr; k++) {
        psEncCtrl->Gains_Q16[k] = silk_SMULWW(psEncCtrl->Gains_Q16[k], gain_mult_Q16);
        assert(psEncCtrl->Gains_Q16[k] >= 0);
        psEncCtrl->Gains_Q16[k] = silk_ADD_POS_SAT32(psEncCtrl->Gains_Q16[k], gain_add_Q16);
    }

    /************************************************/
    /* Control low-frequency shaping and noise tilt */
    /************************************************/
    /* Less low frequency shaping for noisy inputs */
    strength_Q16 = silk_MUL(SILK_FIX_CONST(LOW_FREQ_SHAPING, 4),
                            silk_SMLAWB(SILK_FIX_CONST(1.0, 12), SILK_FIX_CONST(LOW_QUALITY_LOW_FREQ_SHAPING_DECR, 13),
                                        psEnc->sCmn.input_quality_bands_Q15[0] - SILK_FIX_CONST(1.0, 15)));
    strength_Q16 = silk_RSHIFT(silk_MUL(strength_Q16, psEnc->sCmn.speech_activity_Q8), 8);
    if (psEnc->sCmn.indices.signalType == TYPE_VOICED) {
        /* Reduce low frequencies quantization noise for periodic signals, depending on pitch lag */
        /*f = 400; freqz([1, -0.98 + 2e-4 * f], [1, -0.97 + 7e-4 * f], 2^12, Fs); axis([0, 1000, -10, 1])*/
        int32_t fs_kHz_inv = silk_DIV32_16(SILK_FIX_CONST(0.2, 14), psEnc->sCmn.fs_kHz);
        for (k = 0; k < psEnc->sCmn.nb_subfr; k++) {
            b_Q14 = fs_kHz_inv + silk_DIV32_16(SILK_FIX_CONST(3.0, 14), psEncCtrl->pitchL[k]);
            /* Pack two coefficients in one int32 */
            psEncCtrl->LF_shp_Q14[k] =
                silk_LSHIFT(SILK_FIX_CONST(1.0, 14) - b_Q14 - silk_SMULWB(strength_Q16, b_Q14), 16);
            psEncCtrl->LF_shp_Q14[k] |= (uint16_t)(b_Q14 - SILK_FIX_CONST(1.0, 14));
        }
        assert(
            SILK_FIX_CONST(HARM_HP_NOISE_COEF, 24) <
            SILK_FIX_CONST(0.5, 24)); /* Guarantees that second argument to SMULWB() is within range of an int16_t*/
        Tilt_Q16 = -SILK_FIX_CONST(HP_NOISE_COEF, 16) -
                   silk_SMULWB(SILK_FIX_CONST(1.0, 16) - SILK_FIX_CONST(HP_NOISE_COEF, 16),
                               silk_SMULWB(SILK_FIX_CONST(HARM_HP_NOISE_COEF, 24), psEnc->sCmn.speech_activity_Q8));
    } else {
        b_Q14 = silk_DIV32_16(21299, psEnc->sCmn.fs_kHz); /* 1.3_Q0 = 21299_Q14*/
        /* Pack two coefficients in one int32 */
        psEncCtrl->LF_shp_Q14[0] = silk_LSHIFT(
            SILK_FIX_CONST(1.0, 14) - b_Q14 - silk_SMULWB(strength_Q16, silk_SMULWB(SILK_FIX_CONST(0.6, 16), b_Q14)),
            16);
        psEncCtrl->LF_shp_Q14[0] |= (uint16_t)(b_Q14 - SILK_FIX_CONST(1.0, 14));
        for (k = 1; k < psEnc->sCmn.nb_subfr; k++) {
            psEncCtrl->LF_shp_Q14[k] = psEncCtrl->LF_shp_Q14[0];
        }
        Tilt_Q16 = -SILK_FIX_CONST(HP_NOISE_COEF, 16);
    }

    /****************************/
    /* HARMONIC SHAPING CONTROL */
    /****************************/
    if (USE_HARM_SHAPING && psEnc->sCmn.indices.signalType == TYPE_VOICED) {
        /* More harmonic noise shaping for high bitrates or noisy input */
        HarmShapeGain_Q16 =
            silk_SMLAWB(SILK_FIX_CONST(HARMONIC_SHAPING, 16),
                        SILK_FIX_CONST(1.0, 16) -
                            silk_SMULWB(SILK_FIX_CONST(1.0, 18) - silk_LSHIFT(psEncCtrl->coding_quality_Q14, 4),
                                        psEncCtrl->input_quality_Q14),
                        SILK_FIX_CONST(HIGH_RATE_OR_LOW_QUALITY_HARMONIC_SHAPING, 16));

        /* Less harmonic noise shaping for less periodic signals */
        HarmShapeGain_Q16 =
            silk_SMULWB(silk_LSHIFT(HarmShapeGain_Q16, 1), silk_SQRT_APPROX(silk_LSHIFT(psEnc->LTPCorr_Q15, 15)));
    } else {
        HarmShapeGain_Q16 = 0;
    }

    /*************************/
    /* Smooth over subframes */
    /*************************/
    for (k = 0; k < MAX_NB_SUBFR; k++) {
        psShapeSt->HarmShapeGain_smth_Q16 =
            silk_SMLAWB(psShapeSt->HarmShapeGain_smth_Q16, HarmShapeGain_Q16 - psShapeSt->HarmShapeGain_smth_Q16,
                        SILK_FIX_CONST(SUBFR_SMTH_COEF, 16));
        psShapeSt->Tilt_smth_Q16 = silk_SMLAWB(psShapeSt->Tilt_smth_Q16, Tilt_Q16 - psShapeSt->Tilt_smth_Q16,
                                               SILK_FIX_CONST(SUBFR_SMTH_COEF, 16));

        psEncCtrl->HarmShapeGain_Q14[k] = (int32_t)silk_RSHIFT_ROUND(psShapeSt->HarmShapeGain_smth_Q16, 2);
        psEncCtrl->Tilt_Q14[k] = (int32_t)silk_RSHIFT_ROUND(psShapeSt->Tilt_smth_Q16, 2);
    }
}
//----------------------------------------------------------------------------------------------------------------------

/*************************************************************/
/*      FIXED POINT CORE PITCH ANALYSIS FUNCTION             */
/*************************************************************/
int32_t silk_pitch_analysis_core(                  /* O    Voicing estimate: 0 voiced, 1 unvoiced                      */
    const int16_t            *frame_unscaled,    /* I    Signal of length PE_FRAME_LENGTH_MS*Fs_kHz                  */
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
)
{
    VARDECL( int16_t, frame_8kHz_buf );
    VARDECL( int16_t, frame_4kHz );
    VARDECL( int16_t, frame_scaled );
    int32_t filt_state[ 6 ];
    const int16_t *frame, *frame_8kHz;
    int32_t   i, k, d, j;
    VARDECL( int16_t, C );
    VARDECL( int32_t, xcorr32 );
    const int16_t *target_ptr, *basis_ptr;
    int32_t cross_corr, normalizer, energy, energy_basis, energy_target;
    int32_t   d_srch[ PE_D_SRCH_LENGTH ], Cmax, length_d_srch, length_d_comp, shift;
    VARDECL( int16_t, d_comp );
    int32_t sum, threshold, lag_counter;
    int32_t   CBimax, CBimax_new, CBimax_old, lag, start_lag, end_lag, lag_new;
    int32_t CC[ PE_NB_CBKS_STAGE2_EXT ], CCmax, CCmax_b, CCmax_new_b, CCmax_new;
    VARDECL( silk_pe_stage3_vals, energies_st3 );
    VARDECL( silk_pe_stage3_vals, cross_corr_st3 );
    int32_t   frame_length, frame_length_8kHz, frame_length_4kHz;
    int32_t   sf_length;
    int32_t   min_lag;
    int32_t   max_lag;
    int32_t contour_bias_Q15, diff;
    int32_t   nb_cbk_search, cbk_size;
    int32_t delta_lag_log2_sqr_Q7, lag_log2_Q7, prevLag_log2_Q7, prev_lag_bias_Q13;
    const int8_t *Lag_CB_ptr;
    SAVE_STACK;

    /* Check for valid sampling frequency */
    assert( Fs_kHz == 8 || Fs_kHz == 12 || Fs_kHz == 16 );

    /* Check for valid complexity setting */
    assert( complexity >= SILK_PE_MIN_COMPLEX );
    assert( complexity <= SILK_PE_MAX_COMPLEX );

    assert( search_thres1_Q16 >= 0 && search_thres1_Q16 <= (1<<16) );
    assert( search_thres2_Q13 >= 0 && search_thres2_Q13 <= (1<<13) );

    /* Set up frame lengths max / min lag for the sampling frequency */
    frame_length      = ( PE_LTP_MEM_LENGTH_MS + nb_subfr * PE_SUBFR_LENGTH_MS ) * Fs_kHz;
    frame_length_4kHz = ( PE_LTP_MEM_LENGTH_MS + nb_subfr * PE_SUBFR_LENGTH_MS ) * 4;
    frame_length_8kHz = ( PE_LTP_MEM_LENGTH_MS + nb_subfr * PE_SUBFR_LENGTH_MS ) * 8;
    sf_length         = PE_SUBFR_LENGTH_MS * Fs_kHz;
    min_lag           = PE_MIN_LAG_MS * Fs_kHz;
    max_lag           = PE_MAX_LAG_MS * Fs_kHz - 1;

    /* Downscale input if necessary */
    silk_sum_sqr_shift( &energy, &shift, frame_unscaled, frame_length );
    shift += 3 - silk_CLZ32( energy );        /* at least two bits headroom */
    ALLOC( frame_scaled, frame_length, int16_t );
    if( shift > 0 ) {
        shift = silk_RSHIFT( shift + 1, 1 );
        for( i = 0; i < frame_length; i++ ) {
            frame_scaled[ i ] = silk_RSHIFT( frame_unscaled[ i ], shift );
        }
        frame = frame_scaled;
    } else {
        frame = frame_unscaled;
    }

    ALLOC( frame_8kHz_buf, ( Fs_kHz == 8 ) ? 1 : frame_length_8kHz, int16_t );
    /* Resample from input sampled at Fs_kHz to 8 kHz */
    if( Fs_kHz == 16 ) {
        silk_memset( filt_state, 0, 2 * sizeof( int32_t ) );
        silk_resampler_down2( filt_state, frame_8kHz_buf, frame, frame_length );
        frame_8kHz = frame_8kHz_buf;
    } else if( Fs_kHz == 12 ) {
        silk_memset( filt_state, 0, 6 * sizeof( int32_t ) );
        silk_resampler_down2_3( filt_state, frame_8kHz_buf, frame, frame_length );
        frame_8kHz = frame_8kHz_buf;
    } else {
        assert( Fs_kHz == 8 );
        frame_8kHz = frame;
    }

    /* Decimate again to 4 kHz */
    silk_memset( filt_state, 0, 2 * sizeof( int32_t ) );/* Set state to zero */
    ALLOC( frame_4kHz, frame_length_4kHz, int16_t );
    silk_resampler_down2( filt_state, frame_4kHz, frame_8kHz, frame_length_8kHz );

    /* Low-pass filter */
    for( i = frame_length_4kHz - 1; i > 0; i-- ) {
        frame_4kHz[ i ] = silk_ADD_SAT16( frame_4kHz[ i ], frame_4kHz[ i - 1 ] );
    }

    /******************************************************************************
    * FIRST STAGE, operating in 4 khz
    ******************************************************************************/
    ALLOC( C, nb_subfr * CSTRIDE_8KHZ, int16_t );
    ALLOC( xcorr32, MAX_LAG_4KHZ-MIN_LAG_4KHZ+1, int32_t );
    silk_memset( C, 0, (nb_subfr >> 1) * CSTRIDE_4KHZ * sizeof( int16_t ) );
    target_ptr = &frame_4kHz[ silk_LSHIFT( SF_LENGTH_4KHZ, 2 ) ];
    for( k = 0; k < nb_subfr >> 1; k++ ) {
        /* Check that we are within range of the array */
        assert( target_ptr >= frame_4kHz );
        assert( target_ptr + SF_LENGTH_8KHZ <= frame_4kHz + frame_length_4kHz );

        basis_ptr = target_ptr - MIN_LAG_4KHZ;

        /* Check that we are within range of the array */
        assert( basis_ptr >= frame_4kHz );
        assert( basis_ptr + SF_LENGTH_8KHZ <= frame_4kHz + frame_length_4kHz );

        celt_pitch_xcorr( target_ptr, target_ptr - MAX_LAG_4KHZ, xcorr32, SF_LENGTH_8KHZ, MAX_LAG_4KHZ - MIN_LAG_4KHZ + 1, arch );

        /* Calculate first vector products before loop */
        cross_corr = xcorr32[ MAX_LAG_4KHZ - MIN_LAG_4KHZ ];
        normalizer = silk_inner_prod_aligned( target_ptr, target_ptr, SF_LENGTH_8KHZ, arch );
        normalizer = silk_ADD32( normalizer, silk_inner_prod_aligned( basis_ptr,  basis_ptr, SF_LENGTH_8KHZ, arch ) );
        normalizer = silk_ADD32( normalizer, silk_SMULBB( SF_LENGTH_8KHZ, 4000 ) );

        matrix_ptr( C, k, 0, CSTRIDE_4KHZ ) =
            (int16_t)silk_DIV32_varQ( cross_corr, normalizer, 13 + 1 );                      /* Q13 */

        /* From now on normalizer is computed recursively */
        for( d = MIN_LAG_4KHZ + 1; d <= MAX_LAG_4KHZ; d++ ) {
            basis_ptr--;

            /* Check that we are within range of the array */
            assert( basis_ptr >= frame_4kHz );
            assert( basis_ptr + SF_LENGTH_8KHZ <= frame_4kHz + frame_length_4kHz );

            cross_corr = xcorr32[ MAX_LAG_4KHZ - d ];

            /* Add contribution of new sample and remove contribution from oldest sample */
            normalizer = silk_ADD32( normalizer,
                silk_SMULBB( basis_ptr[ 0 ], basis_ptr[ 0 ] ) -
                silk_SMULBB( basis_ptr[ SF_LENGTH_8KHZ ], basis_ptr[ SF_LENGTH_8KHZ ] ) );

            matrix_ptr( C, k, d - MIN_LAG_4KHZ, CSTRIDE_4KHZ) =
                (int16_t)silk_DIV32_varQ( cross_corr, normalizer, 13 + 1 );                  /* Q13 */
        }
        /* Update target pointer */
        target_ptr += SF_LENGTH_8KHZ;
    }

    /* Combine two subframes into single correlation measure and apply short-lag bias */
    if( nb_subfr == PE_MAX_NB_SUBFR ) {
        for( i = MAX_LAG_4KHZ; i >= MIN_LAG_4KHZ; i-- ) {
            sum = (int32_t)matrix_ptr( C, 0, i - MIN_LAG_4KHZ, CSTRIDE_4KHZ )
                + (int32_t)matrix_ptr( C, 1, i - MIN_LAG_4KHZ, CSTRIDE_4KHZ );               /* Q14 */
            sum = silk_SMLAWB( sum, sum, silk_LSHIFT( -i, 4 ) );                                /* Q14 */
            C[ i - MIN_LAG_4KHZ ] = (int16_t)sum;                                            /* Q14 */
        }
    } else {
        /* Only short-lag bias */
        for( i = MAX_LAG_4KHZ; i >= MIN_LAG_4KHZ; i-- ) {
            sum = silk_LSHIFT( (int32_t)C[ i - MIN_LAG_4KHZ ], 1 );                          /* Q14 */
            sum = silk_SMLAWB( sum, sum, silk_LSHIFT( -i, 4 ) );                                /* Q14 */
            C[ i - MIN_LAG_4KHZ ] = (int16_t)sum;                                            /* Q14 */
        }
    }

    /* Sort */
    length_d_srch = silk_ADD_LSHIFT32( 4, complexity, 1 );
    assert( 3 * length_d_srch <= PE_D_SRCH_LENGTH );
    silk_insertion_sort_decreasing_int16( C, d_srch, CSTRIDE_4KHZ,
                                          length_d_srch );

    /* Escape if correlation is very low already here */
    Cmax = (int32_t)C[ 0 ];                                                    /* Q14 */
    if( Cmax < SILK_FIX_CONST( 0.2, 14 ) ) {
        silk_memset( pitch_out, 0, nb_subfr * sizeof( int32_t ) );
        *LTPCorr_Q15  = 0;
        *lagIndex     = 0;
        *contourIndex = 0;

        return 1;
    }

    threshold = silk_SMULWB( search_thres1_Q16, Cmax );
    for( i = 0; i < length_d_srch; i++ ) {
        /* Convert to 8 kHz indices for the sorted correlation that exceeds the threshold */
        if( C[ i ] > threshold ) {
            d_srch[ i ] = silk_LSHIFT( d_srch[ i ] + MIN_LAG_4KHZ, 1 );
        } else {
            length_d_srch = i;
            break;
        }
    }
    assert( length_d_srch > 0 );

    ALLOC( d_comp, D_COMP_STRIDE, int16_t );
    for( i = D_COMP_MIN; i < D_COMP_MAX; i++ ) {
        d_comp[ i - D_COMP_MIN ] = 0;
    }
    for( i = 0; i < length_d_srch; i++ ) {
        d_comp[ d_srch[ i ] - D_COMP_MIN ] = 1;
    }

    /* Convolution */
    for( i = D_COMP_MAX - 1; i >= MIN_LAG_8KHZ; i-- ) {
        d_comp[ i - D_COMP_MIN ] +=
            d_comp[ i - 1 - D_COMP_MIN ] + d_comp[ i - 2 - D_COMP_MIN ];
    }

    length_d_srch = 0;
    for( i = MIN_LAG_8KHZ; i < MAX_LAG_8KHZ + 1; i++ ) {
        if( d_comp[ i + 1 - D_COMP_MIN ] > 0 ) {
            d_srch[ length_d_srch ] = i;
            length_d_srch++;
        }
    }

    /* Convolution */
    for( i = D_COMP_MAX - 1; i >= MIN_LAG_8KHZ; i-- ) {
        d_comp[ i - D_COMP_MIN ] += d_comp[ i - 1 - D_COMP_MIN ]
            + d_comp[ i - 2 - D_COMP_MIN ] + d_comp[ i - 3 - D_COMP_MIN ];
    }

    length_d_comp = 0;
    for( i = MIN_LAG_8KHZ; i < D_COMP_MAX; i++ ) {
        if( d_comp[ i - D_COMP_MIN ] > 0 ) {
            d_comp[ length_d_comp ] = i - 2;
            length_d_comp++;
        }
    }

    /**********************************************************************************
    ** SECOND STAGE, operating at 8 kHz, on lag sections with high correlation
    *************************************************************************************/

    /*********************************************************************************
    * Find energy of each subframe projected onto its history, for a range of delays
    *********************************************************************************/
    silk_memset( C, 0, nb_subfr * CSTRIDE_8KHZ * sizeof( int16_t ) );

    target_ptr = &frame_8kHz[ PE_LTP_MEM_LENGTH_MS * 8 ];
    for( k = 0; k < nb_subfr; k++ ) {

        /* Check that we are within range of the array */
        assert( target_ptr >= frame_8kHz );
        assert( target_ptr + SF_LENGTH_8KHZ <= frame_8kHz + frame_length_8kHz );

        energy_target = silk_ADD32( silk_inner_prod_aligned( target_ptr, target_ptr, SF_LENGTH_8KHZ, arch ), 1 );
        for( j = 0; j < length_d_comp; j++ ) {
            d = d_comp[ j ];
            basis_ptr = target_ptr - d;

            /* Check that we are within range of the array */
            assert( basis_ptr >= frame_8kHz );
            assert( basis_ptr + SF_LENGTH_8KHZ <= frame_8kHz + frame_length_8kHz );

            cross_corr = silk_inner_prod_aligned( target_ptr, basis_ptr, SF_LENGTH_8KHZ, arch );
            if( cross_corr > 0 ) {
                energy_basis = silk_inner_prod_aligned( basis_ptr, basis_ptr, SF_LENGTH_8KHZ, arch );
                matrix_ptr( C, k, d - ( MIN_LAG_8KHZ - 2 ), CSTRIDE_8KHZ ) =
                    (int16_t)silk_DIV32_varQ( cross_corr,
                                                 silk_ADD32( energy_target,
                                                             energy_basis ),
                                                 13 + 1 );                                      /* Q13 */
            } else {
                matrix_ptr( C, k, d - ( MIN_LAG_8KHZ - 2 ), CSTRIDE_8KHZ ) = 0;
            }
        }
        target_ptr += SF_LENGTH_8KHZ;
    }

    /* search over lag range and lags codebook */
    /* scale factor for lag codebook, as a function of center lag */

    CCmax   = silk_int32_MIN;
    CCmax_b = silk_int32_MIN;

    CBimax = 0; /* To avoid returning undefined lag values */
    lag = -1;   /* To check if lag with strong enough correlation has been found */

    if( prevLag > 0 ) {
        if( Fs_kHz == 12 ) {
            prevLag = silk_DIV32_16( silk_LSHIFT( prevLag, 1 ), 3 );
        } else if( Fs_kHz == 16 ) {
            prevLag = silk_RSHIFT( prevLag, 1 );
        }
        prevLag_log2_Q7 = silk_lin2log( (int32_t)prevLag );
    } else {
        prevLag_log2_Q7 = 0;
    }
    assert( search_thres2_Q13 == silk_SAT16( search_thres2_Q13 ) );
    /* Set up stage 2 codebook based on number of subframes */
    if( nb_subfr == PE_MAX_NB_SUBFR ) {
        cbk_size   = PE_NB_CBKS_STAGE2_EXT;
        Lag_CB_ptr = &silk_CB_lags_stage2[ 0 ][ 0 ];
        if( Fs_kHz == 8 && complexity > SILK_PE_MIN_COMPLEX ) {
            /* If input is 8 khz use a larger codebook here because it is last stage */
            nb_cbk_search = PE_NB_CBKS_STAGE2_EXT;
        } else {
            nb_cbk_search = PE_NB_CBKS_STAGE2;
        }
    } else {
        cbk_size       = PE_NB_CBKS_STAGE2_10MS;
        Lag_CB_ptr     = &silk_CB_lags_stage2_10_ms[ 0 ][ 0 ];
        nb_cbk_search  = PE_NB_CBKS_STAGE2_10MS;
    }

    for( k = 0; k < length_d_srch; k++ ) {
        d = d_srch[ k ];
        for( j = 0; j < nb_cbk_search; j++ ) {
            CC[ j ] = 0;
            for( i = 0; i < nb_subfr; i++ ) {
                int32_t d_subfr;
                /* Try all codebooks */
                d_subfr = d + matrix_ptr( Lag_CB_ptr, i, j, cbk_size );
                CC[ j ] = CC[ j ]
                    + (int32_t)matrix_ptr( C, i,
                                              d_subfr - ( MIN_LAG_8KHZ - 2 ),
                                              CSTRIDE_8KHZ );
            }
        }
        /* Find best codebook */
        CCmax_new = silk_int32_MIN;
        CBimax_new = 0;
        for( i = 0; i < nb_cbk_search; i++ ) {
            if( CC[ i ] > CCmax_new ) {
                CCmax_new = CC[ i ];
                CBimax_new = i;
            }
        }

        /* Bias towards shorter lags */
        lag_log2_Q7 = silk_lin2log( d ); /* Q7 */
        assert( lag_log2_Q7 == silk_SAT16( lag_log2_Q7 ) );
        assert( nb_subfr * SILK_FIX_CONST( PE_SHORTLAG_BIAS, 13 ) == silk_SAT16( nb_subfr * SILK_FIX_CONST( PE_SHORTLAG_BIAS, 13 ) ) );
        CCmax_new_b = CCmax_new - silk_RSHIFT( silk_SMULBB( nb_subfr * SILK_FIX_CONST( PE_SHORTLAG_BIAS, 13 ), lag_log2_Q7 ), 7 ); /* Q13 */

        /* Bias towards previous lag */
        assert( nb_subfr * SILK_FIX_CONST( PE_PREVLAG_BIAS, 13 ) == silk_SAT16( nb_subfr * SILK_FIX_CONST( PE_PREVLAG_BIAS, 13 ) ) );
        if( prevLag > 0 ) {
            delta_lag_log2_sqr_Q7 = lag_log2_Q7 - prevLag_log2_Q7;
            assert( delta_lag_log2_sqr_Q7 == silk_SAT16( delta_lag_log2_sqr_Q7 ) );
            delta_lag_log2_sqr_Q7 = silk_RSHIFT( silk_SMULBB( delta_lag_log2_sqr_Q7, delta_lag_log2_sqr_Q7 ), 7 );
            prev_lag_bias_Q13 = silk_RSHIFT( silk_SMULBB( nb_subfr * SILK_FIX_CONST( PE_PREVLAG_BIAS, 13 ), *LTPCorr_Q15 ), 15 ); /* Q13 */
            prev_lag_bias_Q13 = silk_DIV32( silk_MUL( prev_lag_bias_Q13, delta_lag_log2_sqr_Q7 ), delta_lag_log2_sqr_Q7 + SILK_FIX_CONST( 0.5, 7 ) );
            CCmax_new_b -= prev_lag_bias_Q13; /* Q13 */
        }

        if( CCmax_new_b > CCmax_b                                   &&  /* Find maximum biased correlation                  */
            CCmax_new > silk_SMULBB( nb_subfr, search_thres2_Q13 )  &&  /* Correlation needs to be high enough to be voiced */
            silk_CB_lags_stage2[ 0 ][ CBimax_new ] <= MIN_LAG_8KHZ      /* Lag must be in range                             */
         ) {
            CCmax_b = CCmax_new_b;
            CCmax   = CCmax_new;
            lag     = d;
            CBimax  = CBimax_new;
        }
    }

    if( lag == -1 ) {
        /* No suitable candidate found */
        silk_memset( pitch_out, 0, nb_subfr * sizeof( int32_t ) );
        *LTPCorr_Q15  = 0;
        *lagIndex     = 0;
        *contourIndex = 0;

        return 1;
    }

    /* Output normalized correlation */
    *LTPCorr_Q15 = (int32_t)silk_LSHIFT( silk_DIV32_16( CCmax, nb_subfr ), 2 );
    assert( *LTPCorr_Q15 >= 0 );

    if( Fs_kHz > 8 ) {
        /* Search in original signal */

        CBimax_old = CBimax;
        /* Compensate for decimation */
        assert( lag == silk_SAT16( lag ) );
        if( Fs_kHz == 12 ) {
            lag = silk_RSHIFT( silk_SMULBB( lag, 3 ), 1 );
        } else if( Fs_kHz == 16 ) {
            lag = silk_LSHIFT( lag, 1 );
        } else {
            lag = silk_SMULBB( lag, 3 );
        }

        lag = silk_LIMIT_int( lag, min_lag, max_lag );
        start_lag = silk_max_int( lag - 2, min_lag );
        end_lag   = silk_min_int( lag + 2, max_lag );
        lag_new   = lag;                                    /* to avoid undefined lag */
        CBimax    = 0;                                      /* to avoid undefined lag */

        CCmax = silk_int32_MIN;
        /* pitch lags according to second stage */
        for( k = 0; k < nb_subfr; k++ ) {
            pitch_out[ k ] = lag + 2 * silk_CB_lags_stage2[ k ][ CBimax_old ];
        }

        /* Set up codebook parameters according to complexity setting and frame length */
        if( nb_subfr == PE_MAX_NB_SUBFR ) {
            nb_cbk_search   = (int32_t)silk_nb_cbk_searchs_stage3[ complexity ];
            cbk_size        = PE_NB_CBKS_STAGE3_MAX;
            Lag_CB_ptr      = &silk_CB_lags_stage3[ 0 ][ 0 ];
        } else {
            nb_cbk_search   = PE_NB_CBKS_STAGE3_10MS;
            cbk_size        = PE_NB_CBKS_STAGE3_10MS;
            Lag_CB_ptr      = &silk_CB_lags_stage3_10_ms[ 0 ][ 0 ];
        }

        /* Calculate the correlations and energies needed in stage 3 */
        ALLOC( energies_st3, nb_subfr * nb_cbk_search, silk_pe_stage3_vals );
        ALLOC( cross_corr_st3, nb_subfr * nb_cbk_search, silk_pe_stage3_vals );
        silk_P_Ana_calc_corr_st3(  cross_corr_st3, frame, start_lag, sf_length, nb_subfr, complexity, arch );
        silk_P_Ana_calc_energy_st3( energies_st3, frame, start_lag, sf_length, nb_subfr, complexity, arch );

        lag_counter = 0;
        assert( lag == silk_SAT16( lag ) );
        contour_bias_Q15 = silk_DIV32_16( SILK_FIX_CONST( PE_FLATCONTOUR_BIAS, 15 ), lag );

        target_ptr = &frame[ PE_LTP_MEM_LENGTH_MS * Fs_kHz ];
        energy_target = silk_ADD32( silk_inner_prod_aligned( target_ptr, target_ptr, nb_subfr * sf_length, arch ), 1 );
        for( d = start_lag; d <= end_lag; d++ ) {
            for( j = 0; j < nb_cbk_search; j++ ) {
                cross_corr = 0;
                energy     = energy_target;
                for( k = 0; k < nb_subfr; k++ ) {
                    cross_corr = silk_ADD32( cross_corr,
                        matrix_ptr( cross_corr_st3, k, j,
                                    nb_cbk_search )[ lag_counter ] );
                    energy     = silk_ADD32( energy,
                        matrix_ptr( energies_st3, k, j,
                                    nb_cbk_search )[ lag_counter ] );
                    assert( energy >= 0 );
                }
                if( cross_corr > 0 ) {
                    CCmax_new = silk_DIV32_varQ( cross_corr, energy, 13 + 1 );          /* Q13 */
                    /* Reduce depending on flatness of contour */
                    diff = silk_int16_MAX - silk_MUL( contour_bias_Q15, j );            /* Q15 */
                    assert( diff == silk_SAT16( diff ) );
                    CCmax_new = silk_SMULWB( CCmax_new, diff );                         /* Q14 */
                } else {
                    CCmax_new = 0;
                }

                if( CCmax_new > CCmax && ( d + silk_CB_lags_stage3[ 0 ][ j ] ) <= max_lag ) {
                    CCmax   = CCmax_new;
                    lag_new = d;
                    CBimax  = j;
                }
            }
            lag_counter++;
        }

        for( k = 0; k < nb_subfr; k++ ) {
            pitch_out[ k ] = lag_new + matrix_ptr( Lag_CB_ptr, k, CBimax, cbk_size );
            pitch_out[ k ] = silk_LIMIT( pitch_out[ k ], min_lag, PE_MAX_LAG_MS * Fs_kHz );
        }
        *lagIndex = (int16_t)( lag_new - min_lag);
        *contourIndex = (int8_t)CBimax;
    } else {        /* Fs_kHz == 8 */
        /* Save Lags */
        for( k = 0; k < nb_subfr; k++ ) {
            pitch_out[ k ] = lag + matrix_ptr( Lag_CB_ptr, k, CBimax, cbk_size );
            pitch_out[ k ] = silk_LIMIT( pitch_out[ k ], MIN_LAG_8KHZ, PE_MAX_LAG_MS * 8 );
        }
        *lagIndex = (int16_t)( lag - MIN_LAG_8KHZ );
        *contourIndex = (int8_t)CBimax;
    }
    assert( *lagIndex >= 0 );
    /* return as voiced */

    return 0;
}
//----------------------------------------------------------------------------------------------------------------------

/***********************************************************************
 * Calculates the correlations used in stage 3 search. In order to cover
 * the whole lag codebook for all the searched offset lags (lag +- 2),
 * the following correlations are needed in each sub frame:
 *
 * sf1: lag range [-8,...,7] total 16 correlations
 * sf2: lag range [-4,...,4] total 9 correlations
 * sf3: lag range [-3,....4] total 8 correltions
 * sf4: lag range [-6,....8] total 15 correlations
 *
 * In total 48 correlations. The direct implementation computed in worst
 * case 4*12*5 = 240 correlations, but more likely around 120.
 ***********************************************************************/
static void silk_P_Ana_calc_corr_st3(
    silk_pe_stage3_vals cross_corr_st3[],              /* O 3 DIM correlation array */
    const int16_t  frame[],                         /* I vector to correlate         */
    int32_t          start_lag,                       /* I lag offset to search around */
    int32_t          sf_length,                       /* I length of a 5 ms subframe   */
    int32_t          nb_subfr,                        /* I number of subframes         */
    int32_t          complexity,                      /* I Complexity setting          */
    int               arch                             /* I Run-time architecture       */
)
{
    const int16_t *target_ptr;
    int32_t   i, j, k, lag_counter, lag_low, lag_high;
    int32_t   nb_cbk_search, delta, idx, cbk_size;
    VARDECL( int32_t, scratch_mem );
    VARDECL( int32_t, xcorr32 );
    const int8_t *Lag_range_ptr, *Lag_CB_ptr;
    SAVE_STACK;

    assert( complexity >= SILK_PE_MIN_COMPLEX );
    assert( complexity <= SILK_PE_MAX_COMPLEX );

    if( nb_subfr == PE_MAX_NB_SUBFR ) {
        Lag_range_ptr = &silk_Lag_range_stage3[ complexity ][ 0 ][ 0 ];
        Lag_CB_ptr    = &silk_CB_lags_stage3[ 0 ][ 0 ];
        nb_cbk_search = silk_nb_cbk_searchs_stage3[ complexity ];
        cbk_size      = PE_NB_CBKS_STAGE3_MAX;
    } else {
        assert( nb_subfr == PE_MAX_NB_SUBFR >> 1);
        Lag_range_ptr = &silk_Lag_range_stage3_10_ms[ 0 ][ 0 ];
        Lag_CB_ptr    = &silk_CB_lags_stage3_10_ms[ 0 ][ 0 ];
        nb_cbk_search = PE_NB_CBKS_STAGE3_10MS;
        cbk_size      = PE_NB_CBKS_STAGE3_10MS;
    }
    ALLOC( scratch_mem, SCRATCH_SIZE, int32_t );
    ALLOC( xcorr32, SCRATCH_SIZE, int32_t );

    target_ptr = &frame[ silk_LSHIFT( sf_length, 2 ) ]; /* Pointer to middle of frame */
    for( k = 0; k < nb_subfr; k++ ) {
        lag_counter = 0;

        /* Calculate the correlations for each subframe */
        lag_low  = matrix_ptr( Lag_range_ptr, k, 0, 2 );
        lag_high = matrix_ptr( Lag_range_ptr, k, 1, 2 );
        assert(lag_high-lag_low+1 <= SCRATCH_SIZE);
        celt_pitch_xcorr( target_ptr, target_ptr - start_lag - lag_high, xcorr32, sf_length, lag_high - lag_low + 1, arch );
        for( j = lag_low; j <= lag_high; j++ ) {
            assert( lag_counter < SCRATCH_SIZE );
            scratch_mem[ lag_counter ] = xcorr32[ lag_high - j ];
            lag_counter++;
        }

        delta = matrix_ptr( Lag_range_ptr, k, 0, 2 );
        for( i = 0; i < nb_cbk_search; i++ ) {
            /* Fill out the 3 dim array that stores the correlations for */
            /* each code_book vector for each start lag */
            idx = matrix_ptr( Lag_CB_ptr, k, i, cbk_size ) - delta;
            for( j = 0; j < PE_NB_STAGE3_LAGS; j++ ) {
                assert( idx + j < SCRATCH_SIZE );
                assert( idx + j < lag_counter );
                matrix_ptr( cross_corr_st3, k, i, nb_cbk_search )[ j ] =
                    scratch_mem[ idx + j ];
            }
        }
        target_ptr += sf_length;
    }
}
//----------------------------------------------------------------------------------------------------------------------

/********************************************************************/
/* Calculate the energies for first two subframes. The energies are */
/* calculated recursively.                                          */
/********************************************************************/
static void silk_P_Ana_calc_energy_st3(
    silk_pe_stage3_vals energies_st3[],                 /* O 3 DIM energy array */
    const int16_t  frame[],                          /* I vector to calc energy in    */
    int32_t          start_lag,                        /* I lag offset to search around */
    int32_t          sf_length,                        /* I length of one 5 ms subframe */
    int32_t          nb_subfr,                         /* I number of subframes         */
    int32_t          complexity,                       /* I Complexity setting          */
    int               arch                              /* I Run-time architecture       */
)
{
    const int16_t *target_ptr, *basis_ptr;
    int32_t energy;
    int32_t   k, i, j, lag_counter;
    int32_t   nb_cbk_search, delta, idx, cbk_size, lag_diff;
    VARDECL( int32_t, scratch_mem );
    const int8_t *Lag_range_ptr, *Lag_CB_ptr;
    SAVE_STACK;

    assert( complexity >= SILK_PE_MIN_COMPLEX );
    assert( complexity <= SILK_PE_MAX_COMPLEX );

    if( nb_subfr == PE_MAX_NB_SUBFR ) {
        Lag_range_ptr = &silk_Lag_range_stage3[ complexity ][ 0 ][ 0 ];
        Lag_CB_ptr    = &silk_CB_lags_stage3[ 0 ][ 0 ];
        nb_cbk_search = silk_nb_cbk_searchs_stage3[ complexity ];
        cbk_size      = PE_NB_CBKS_STAGE3_MAX;
    } else {
        assert( nb_subfr == PE_MAX_NB_SUBFR >> 1);
        Lag_range_ptr = &silk_Lag_range_stage3_10_ms[ 0 ][ 0 ];
        Lag_CB_ptr    = &silk_CB_lags_stage3_10_ms[ 0 ][ 0 ];
        nb_cbk_search = PE_NB_CBKS_STAGE3_10MS;
        cbk_size      = PE_NB_CBKS_STAGE3_10MS;
    }
    ALLOC( scratch_mem, SCRATCH_SIZE, int32_t );

    target_ptr = &frame[ silk_LSHIFT( sf_length, 2 ) ];
    for( k = 0; k < nb_subfr; k++ ) {
        lag_counter = 0;

        /* Calculate the energy for first lag */
        basis_ptr = target_ptr - ( start_lag + matrix_ptr( Lag_range_ptr, k, 0, 2 ) );
        energy = silk_inner_prod_aligned( basis_ptr, basis_ptr, sf_length, arch );
        assert( energy >= 0 );
        scratch_mem[ lag_counter ] = energy;
        lag_counter++;

        lag_diff = ( matrix_ptr( Lag_range_ptr, k, 1, 2 ) -  matrix_ptr( Lag_range_ptr, k, 0, 2 ) + 1 );
        for( i = 1; i < lag_diff; i++ ) {
            /* remove part outside new window */
            energy -= silk_SMULBB( basis_ptr[ sf_length - i ], basis_ptr[ sf_length - i ] );
            assert( energy >= 0 );

            /* add part that comes into window */
            energy = silk_ADD_SAT32( energy, silk_SMULBB( basis_ptr[ -i ], basis_ptr[ -i ] ) );
            assert( energy >= 0 );
            assert( lag_counter < SCRATCH_SIZE );
            scratch_mem[ lag_counter ] = energy;
            lag_counter++;
        }

        delta = matrix_ptr( Lag_range_ptr, k, 0, 2 );
        for( i = 0; i < nb_cbk_search; i++ ) {
            /* Fill out the 3 dim array that stores the correlations for    */
            /* each code_book vector for each start lag                     */
            idx = matrix_ptr( Lag_CB_ptr, k, i, cbk_size ) - delta;
            for( j = 0; j < PE_NB_STAGE3_LAGS; j++ ) {
                assert( idx + j < SCRATCH_SIZE );
                assert( idx + j < lag_counter );
                matrix_ptr( energies_st3, k, i, nb_cbk_search )[ j ] =
                    scratch_mem[ idx + j ];
                assert(
                    matrix_ptr( energies_st3, k, i, nb_cbk_search )[ j ] >= 0 );
            }
        }
        target_ptr += sf_length;
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Processing of gains */
void silk_process_gains_FIX(silk_encoder_state_FIX *psEnc,       /* I/O  Encoder state       */
                            silk_encoder_control_FIX *psEncCtrl, /* I/O  Encoder control */
                            int32_t condCoding                   /* I    The type of conditional coding to use                   */
) {
    silk_shape_state_FIX *psShapeSt = &psEnc->sShape;
    int32_t k;
    int32_t s_Q16, InvMaxSqrVal_Q16, gain, gain_squared, ResNrg, ResNrgPart, quant_offset_Q10;

    /* Gain reduction when LTP coding gain is high */
    if (psEnc->sCmn.indices.signalType == TYPE_VOICED) {
        /*s = -0.5f * silk_sigmoid( 0.25f * ( psEncCtrl->LTPredCodGain - 12.0f ) ); */
        s_Q16 = -silk_sigm_Q15(silk_RSHIFT_ROUND(psEncCtrl->LTPredCodGain_Q7 - SILK_FIX_CONST(12.0, 7), 4));
        for (k = 0; k < psEnc->sCmn.nb_subfr; k++) {
            psEncCtrl->Gains_Q16[k] = silk_SMLAWB(psEncCtrl->Gains_Q16[k], psEncCtrl->Gains_Q16[k], s_Q16);
        }
    }

    /* Limit the quantized signal */
    /* InvMaxSqrVal = pow( 2.0f, 0.33f * ( 21.0f - SNR_dB ) ) / subfr_length; */
    InvMaxSqrVal_Q16 = silk_DIV32_16(
        silk_log2lin(silk_SMULWB(SILK_FIX_CONST(21 + 16 / 0.33, 7) - psEnc->sCmn.SNR_dB_Q7, SILK_FIX_CONST(0.33, 16))),
        psEnc->sCmn.subfr_length);

    for (k = 0; k < psEnc->sCmn.nb_subfr; k++) {
        /* Soft limit on ratio residual energy and squared gains */
        ResNrg = psEncCtrl->ResNrg[k];
        ResNrgPart = silk_SMULWW(ResNrg, InvMaxSqrVal_Q16);
        if (psEncCtrl->ResNrgQ[k] > 0) {
            ResNrgPart = silk_RSHIFT_ROUND(ResNrgPart, psEncCtrl->ResNrgQ[k]);
        } else {
            if (ResNrgPart >= silk_RSHIFT(silk_int32_MAX, -psEncCtrl->ResNrgQ[k])) {
                ResNrgPart = silk_int32_MAX;
            } else {
                ResNrgPart = silk_LSHIFT(ResNrgPart, -psEncCtrl->ResNrgQ[k]);
            }
        }
        gain = psEncCtrl->Gains_Q16[k];
        gain_squared = silk_ADD_SAT32(ResNrgPart, silk_SMMUL(gain, gain));
        if (gain_squared < silk_int16_MAX) {
            /* recalculate with higher precision */
            gain_squared = silk_SMLAWW(silk_LSHIFT(ResNrgPart, 16), gain, gain);
            assert(gain_squared > 0);
            gain = silk_SQRT_APPROX(gain_squared); /* Q8   */
            gain = silk_min(gain, silk_int32_MAX >> 8);
            psEncCtrl->Gains_Q16[k] = silk_LSHIFT_SAT32(gain, 8); /* Q16  */
        } else {
            gain = silk_SQRT_APPROX(gain_squared); /* Q0   */
            gain = silk_min(gain, silk_int32_MAX >> 16);
            psEncCtrl->Gains_Q16[k] = silk_LSHIFT_SAT32(gain, 16); /* Q16  */
        }
    }

    /* Save unquantized gains and gain Index */
    silk_memcpy(psEncCtrl->GainsUnq_Q16, psEncCtrl->Gains_Q16, psEnc->sCmn.nb_subfr * sizeof(int32_t));
    psEncCtrl->lastGainIndexPrev = psShapeSt->LastGainIndex;

    /* Quantize gains */
    silk_gains_quant(psEnc->sCmn.indices.GainsIndices, psEncCtrl->Gains_Q16, &psShapeSt->LastGainIndex,
                     condCoding == CODE_CONDITIONALLY, psEnc->sCmn.nb_subfr);

    /* Set quantizer offset for voiced signals. Larger offset when LTP coding gain is low or tilt is high (ie low-pass)
     */
    if (psEnc->sCmn.indices.signalType == TYPE_VOICED) {
        if (psEncCtrl->LTPredCodGain_Q7 + silk_RSHIFT(psEnc->sCmn.input_tilt_Q15, 8) > SILK_FIX_CONST(1.0, 7)) {
            psEnc->sCmn.indices.quantOffsetType = 0;
        } else {
            psEnc->sCmn.indices.quantOffsetType = 1;
        }
    }

    /* Quantizer boundary adjustment */
    quant_offset_Q10 =
        silk_Quantization_Offsets_Q10[psEnc->sCmn.indices.signalType >> 1][psEnc->sCmn.indices.quantOffsetType];
    psEncCtrl->Lambda_Q10 =
        SILK_FIX_CONST(LAMBDA_OFFSET, 10) +
        silk_SMULBB(SILK_FIX_CONST(LAMBDA_DELAYED_DECISIONS, 10), psEnc->sCmn.nStatesDelayedDecision) +
        silk_SMULWB(SILK_FIX_CONST(LAMBDA_SPEECH_ACT, 18), psEnc->sCmn.speech_activity_Q8) +
        silk_SMULWB(SILK_FIX_CONST(LAMBDA_INPUT_QUALITY, 12), psEncCtrl->input_quality_Q14) +
        silk_SMULWB(SILK_FIX_CONST(LAMBDA_CODING_QUALITY, 12), psEncCtrl->coding_quality_Q14) +
        silk_SMULWB(SILK_FIX_CONST(LAMBDA_QUANT_OFFSET, 16), quant_offset_Q10);

    assert(psEncCtrl->Lambda_Q10 > 0);
    assert(psEncCtrl->Lambda_Q10 < SILK_FIX_CONST(2, 10));
}
//----------------------------------------------------------------------------------------------------------------------

/* Add noise to matrix diagonal */
void silk_regularize_correlations_FIX(int32_t *XX,   /* I/O  Correlation matrices   */
                                      int32_t *xx,   /* I/O  Correlation values   */
                                      int32_t noise, /* I    Noise to add */
                                      int32_t D      /* I    Dimension of XX      */
) {
    int32_t i;
    for (i = 0; i < D; i++) {
        matrix_ptr(&XX[0], i, i, D) = silk_ADD32(matrix_ptr(&XX[0], i, i, D), noise);
    }
    xx[0] += noise;
}
//----------------------------------------------------------------------------------------------------------------------

/* Calculates residual energies of input subframes where all subframes have LPC_order   */
/* of preceding samples                                                                 */
void silk_residual_energy_FIX(int32_t nrgs[MAX_NB_SUBFR],        /* O    Residual energy per subframe        */
                              int32_t nrgsQ[MAX_NB_SUBFR],       /* O    Q value per subframe       */
                              const int16_t x[],                 /* I    Input signal                 */
                              int16_t a_Q12[2][MAX_LPC_ORDER],   /* I    AR coefs for each frame half   */
                              const int32_t gains[MAX_NB_SUBFR], /* I    Quantization gains */
                              const int32_t subfr_length,        /* I    Subframe length        */
                              const int32_t nb_subfr,            /* I    Number of subframes            */
                              const int32_t LPC_order,           /* I    LPC order           */
                              int arch                           /* I    Run-time architecture                           */
) {
    int32_t offset, i, j, rshift, lz1, lz2;
    int16_t *LPC_res_ptr;
    VARDECL(int16_t, LPC_res);
    const int16_t *x_ptr;
    int32_t tmp32;
    SAVE_STACK;

    x_ptr = x;
    offset = LPC_order + subfr_length;

    /* Filter input to create the LPC residual for each frame half, and measure subframe energies */
    ALLOC(LPC_res, (MAX_NB_SUBFR >> 1) * offset, int16_t);
    assert((nb_subfr >> 1) * (MAX_NB_SUBFR >> 1) == nb_subfr);
    for (i = 0; i < nb_subfr >> 1; i++) {
        /* Calculate half frame LPC residual signal including preceding samples */
        silk_LPC_analysis_filter(LPC_res, x_ptr, a_Q12[i], (MAX_NB_SUBFR >> 1) * offset, LPC_order, arch);

        /* Point to first subframe of the just calculated LPC residual signal */
        LPC_res_ptr = LPC_res + LPC_order;
        for (j = 0; j < (MAX_NB_SUBFR >> 1); j++) {
            /* Measure subframe energy */
            silk_sum_sqr_shift(&nrgs[i * (MAX_NB_SUBFR >> 1) + j], &rshift, LPC_res_ptr, subfr_length);

            /* Set Q values for the measured energy */
            nrgsQ[i * (MAX_NB_SUBFR >> 1) + j] = -rshift;

            /* Move to next subframe */
            LPC_res_ptr += offset;
        }
        /* Move to next frame half */
        x_ptr += (MAX_NB_SUBFR >> 1) * offset;
    }

    /* Apply the squared subframe gains */
    for (i = 0; i < nb_subfr; i++) {
        /* Fully upscale gains and energies */
        lz1 = silk_CLZ32(nrgs[i]) - 1;
        lz2 = silk_CLZ32(gains[i]) - 1;

        tmp32 = silk_LSHIFT32(gains[i], lz2);

        /* Find squared gains */
        tmp32 = silk_SMMUL(tmp32, tmp32); /* Q( 2 * lz2 - 32 )*/

        /* Scale energies */
        nrgs[i] = silk_SMMUL(tmp32, silk_LSHIFT32(nrgs[i], lz1)); /* Q( nrgsQ[ i ] + lz1 + 2 * lz2 - 32 - 32 )*/
        nrgsQ[i] += lz1 + 2 * lz2 - 32 - 32;
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Residual energy: nrg = wxx - 2 * wXx * c + c' * wXX * c */
int32_t silk_residual_energy16_covar_FIX(const int16_t *c,   /* I    Prediction vector   */
                                         const int32_t *wXX, /* I    Correlation matrix */
                                         const int32_t *wXx, /* I    Correlation vector */
                                         int32_t wxx,        /* I    Signal energy        */
                                         int32_t D,          /* I    Dimension          */
                                         int32_t cQ          /* I    Q value for c vector 0 - 15          */
) {
    int32_t i, j, lshifts, Qxtra;
    int32_t c_max, w_max, tmp, tmp2, nrg;
    int32_t cn[MAX_MATRIX_SIZE];
    const int32_t *pRow;

    /* Safety checks */
    assert(D >= 0);
    assert(D <= 16);
    assert(cQ > 0);
    assert(cQ < 16);

    lshifts = 16 - cQ;
    Qxtra = lshifts;

    c_max = 0;
    for (i = 0; i < D; i++) {
        c_max = silk_max_32(c_max, silk_abs((int32_t)c[i]));
    }
    Qxtra = silk_min_int(Qxtra, silk_CLZ32(c_max) - 17);

    w_max = silk_max_32(wXX[0], wXX[D * D - 1]);
    Qxtra = silk_min_int(Qxtra, silk_CLZ32(silk_MUL(D, silk_RSHIFT(silk_SMULWB(w_max, c_max), 4))) - 5);
    Qxtra = silk_max_int(Qxtra, 0);
    for (i = 0; i < D; i++) {
        cn[i] = silk_LSHIFT((int32_t)c[i], Qxtra);
        assert(silk_abs(cn[i]) <= (silk_int16_MAX + 1)); /* Check that silk_SMLAWB can be used */
    }
    lshifts -= Qxtra;

    /* Compute wxx - 2 * wXx * c */
    tmp = 0;
    for (i = 0; i < D; i++) {
        tmp = silk_SMLAWB(tmp, wXx[i], cn[i]);
    }
    nrg = silk_RSHIFT(wxx, 1 + lshifts) - tmp; /* Q: -lshifts - 1 */

    /* Add c' * wXX * c, assuming wXX is symmetric */
    tmp2 = 0;
    for (i = 0; i < D; i++) {
        tmp = 0;
        pRow = &wXX[i * D];
        for (j = i + 1; j < D; j++) {
            tmp = silk_SMLAWB(tmp, pRow[j], cn[j]);
        }
        tmp = silk_SMLAWB(tmp, silk_RSHIFT(pRow[i], 1), cn[i]);
        tmp2 = silk_SMLAWB(tmp2, tmp, cn[i]);
    }
    nrg = silk_ADD_LSHIFT32(nrg, tmp2, lshifts); /* Q: -lshifts - 1 */

    /* Keep one bit free always, because we add them for LSF interpolation */
    if (nrg < 1) {
        nrg = 1;
    } else if (nrg > silk_RSHIFT(silk_int32_MAX, lshifts + 2)) {
        nrg = silk_int32_MAX >> 1;
    } else {
        nrg = silk_LSHIFT(nrg, lshifts + 1); /* Q0 */
    }
    return nrg;
}
//----------------------------------------------------------------------------------------------------------------------

/* Faster than schur64(), but much less accurate.                       */
/* uses SMLAWB(), requiring armv5E and higher.                          */
int32_t silk_schur(                    /* O    Returns residual energy                                     */
                   int16_t *rc_Q15,    /* O    reflection coefficients [order] Q15                         */
                   const int32_t *c,   /* I    correlations [order+1]                                      */
                   const int32_t order /* I    prediction order                                            */
) {
    int32_t k, n, lz;
    int32_t C[SILK_MAX_ORDER_LPC + 1][2];
    int32_t Ctmp1, Ctmp2, rc_tmp_Q15;

    assert(order >= 0 && order <= SILK_MAX_ORDER_LPC);

    /* Get number of leading zeros */
    lz = silk_CLZ32(c[0]);

    /* Copy correlations and adjust level to Q30 */
    k = 0;
    if (lz < 2) {
        /* lz must be 1, so shift one to the right */
        do {
            C[k][0] = C[k][1] = silk_RSHIFT(c[k], 1);
        } while (++k <= order);
    } else if (lz > 2) {
        /* Shift to the left */
        lz -= 2;
        do {
            C[k][0] = C[k][1] = silk_LSHIFT(c[k], lz);
        } while (++k <= order);
    } else {
        /* No need to shift */
        do {
            C[k][0] = C[k][1] = c[k];
        } while (++k <= order);
    }

    for (k = 0; k < order; k++) {
        /* Check that we won't be getting an unstable rc, otherwise stop here. */
        if (silk_abs_int32(C[k + 1][0]) >= C[0][1]) {
            if (C[k + 1][0] > 0) {
                rc_Q15[k] = -SILK_FIX_CONST(.99f, 15);
            } else {
                rc_Q15[k] = SILK_FIX_CONST(.99f, 15);
            }
            k++;
            break;
        }

        /* Get reflection coefficient */
        rc_tmp_Q15 = -silk_DIV32_16(C[k + 1][0], silk_max_32(silk_RSHIFT(C[0][1], 15), 1));

        /* Clip (shouldn't happen for properly conditioned inputs) */
        rc_tmp_Q15 = silk_SAT16(rc_tmp_Q15);

        /* Store */
        rc_Q15[k] = (int16_t)rc_tmp_Q15;

        /* Update correlations */
        for (n = 0; n < order - k; n++) {
            Ctmp1 = C[n + k + 1][0];
            Ctmp2 = C[n][1];
            C[n + k + 1][0] = silk_SMLAWB(Ctmp1, silk_LSHIFT(Ctmp2, 1), rc_tmp_Q15);
            C[n][1] = silk_SMLAWB(Ctmp2, silk_LSHIFT(Ctmp1, 1), rc_tmp_Q15);
        }
    }

    for (; k < order; k++) {
        rc_Q15[k] = 0;
    }

    /* return residual energy */
    return silk_max_32(1, C[0][1]);
}
//----------------------------------------------------------------------------------------------------------------------

/* Slower than schur(), but more accurate.                              */
/* Uses SMULL(), available on armv4                                     */
int32_t silk_schur64(                   /* O    returns residual energy                                     */
                     int32_t rc_Q16[],  /* O    Reflection coefficients [order] Q16                         */
                     const int32_t c[], /* I    Correlations [order+1]                                      */
                     int32_t order      /* I    Prediction order                                            */
) {
    int32_t k, n;
    int32_t C[SILK_MAX_ORDER_LPC + 1][2];
    int32_t Ctmp1_Q30, Ctmp2_Q30, rc_tmp_Q31;

    assert(order >= 0 && order <= SILK_MAX_ORDER_LPC);

    /* Check for invalid input */
    if (c[0] <= 0) {
        silk_memset(rc_Q16, 0, order * sizeof(int32_t));
        return 0;
    }

    k = 0;
    do {
        C[k][0] = C[k][1] = c[k];
    } while (++k <= order);

    for (k = 0; k < order; k++) {
        /* Check that we won't be getting an unstable rc, otherwise stop here. */
        if (silk_abs_int32(C[k + 1][0]) >= C[0][1]) {
            if (C[k + 1][0] > 0) {
                rc_Q16[k] = -SILK_FIX_CONST(.99f, 16);
            } else {
                rc_Q16[k] = SILK_FIX_CONST(.99f, 16);
            }
            k++;
            break;
        }

        /* Get reflection coefficient: divide two Q30 values and get result in Q31 */
        rc_tmp_Q31 = silk_DIV32_varQ(-C[k + 1][0], C[0][1], 31);

        /* Save the output */
        rc_Q16[k] = silk_RSHIFT_ROUND(rc_tmp_Q31, 15);

        /* Update correlations */
        for (n = 0; n < order - k; n++) {
            Ctmp1_Q30 = C[n + k + 1][0];
            Ctmp2_Q30 = C[n][1];

            /* Multiply and add the highest int32 */
            C[n + k + 1][0] = Ctmp1_Q30 + silk_SMMUL(silk_LSHIFT(Ctmp2_Q30, 1), rc_tmp_Q31);
            C[n][1] = Ctmp2_Q30 + silk_SMMUL(silk_LSHIFT(Ctmp1_Q30, 1), rc_tmp_Q31);
        }
    }

    for (; k < order; k++) {
        rc_Q16[k] = 0;
    }

    return silk_max_32(1, C[0][1]);
}
//----------------------------------------------------------------------------------------------------------------------

/* Copy and multiply a vector by a constant */
void silk_scale_copy_vector16(int16_t *data_out, const int16_t *data_in, int32_t gain_Q16, /* I    Gain in Q16 */
                              const int32_t dataSize                                       /* I    Length                                       */
) {
    int32_t i;
    int32_t tmp32;

    for (i = 0; i < dataSize; i++) {
        tmp32 = silk_SMULWB(gain_Q16, data_in[i]);
        data_out[i] = (int16_t)silk_CHECK_FIT16(tmp32);
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Multiply a vector by a constant */
void silk_scale_vector32_Q26_lshift_18(int32_t *data1,   /* I/O  Q0/Q18   */
                                       int32_t gain_Q26, /* I    Q26 */
                                       int32_t dataSize  /* I    length  */
) {
    int32_t i;

    for (i = 0; i < dataSize; i++) {
        data1[i] = (int32_t)silk_CHECK_FIT32(silk_RSHIFT64(silk_SMULL(data1[i], gain_Q26), 8)); /* OUTPUT: Q18 */
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* sum = for(i=0;i<len;i++)inVec1[i]*inVec2[i];      ---        inner product   */
/* Note for ARM asm:                                                            */
/*        * inVec1 and inVec2 should be at least 2 byte aligned.                */
/*        * len should be positive 16bit integer.                               */
/*        * only when len>6, memory access can be reduced by half.              */
int32_t silk_inner_prod_aligned(const int16_t *const inVec1, /*    I input vector 1 */
                                const int16_t *const inVec2, /*    I input vector 2 */
                                const int32_t len,           /*    I vector lengths */
                                int arch /*    I Run-time architecture              */
) {
    return celt_inner_prod(inVec1, inVec2, len, arch);
}
//----------------------------------------------------------------------------------------------------------------------

int64_t silk_inner_prod16_aligned_64_c(const int16_t *inVec1, /*    I input vector 1 */
                                       const int16_t *inVec2, /*    I input vector 2 */
                                       const int32_t len      /*    I vector lengths */
) {
    int32_t i;
    int64_t sum = 0;
    for (i = 0; i < len; i++) {
        sum = silk_SMLALBB(sum, inVec1[i], inVec2[i]);
    }
    return sum;
}
//----------------------------------------------------------------------------------------------------------------------

/* Autocorrelations for a warped frequency axis */
void silk_warped_autocorrelation_FIX_c(int32_t *corr,             /* O    Result [order + 1]                */
                                       int32_t *scale,            /* O    Scaling of the correlation vector */
                                       const int16_t *input,      /* I    Input data to correlate           */
                                       const int32_t warping_Q16, /* I    Warping coefficient               */
                                       const int32_t length,      /* I    Length of input                   */
                                       const int32_t order        /* I    Correlation order (even)          */
) {
    int32_t n, i, lsh;
    int32_t tmp1_QS, tmp2_QS;
    int32_t *state_QS = (int32_t *)calloc(MAX_SHAPE_LPC_ORDER + 1, sizeof(int32_t));
    int64_t *corr_QC = (int64_t *)calloc(MAX_SHAPE_LPC_ORDER + 1, sizeof(int64_t));

    /* Order must be even */
    assert((order & 1) == 0);
    assert(2 * QS - QC >= 0);

    /* Loop over samples */
    for (n = 0; n < length; n++) {
        tmp1_QS = silk_LSHIFT32((int32_t)input[n], QS);
        /* Loop over allpass sections */
        for (i = 0; i < order; i += 2) {
            /* Output of allpass section */
            tmp2_QS = silk_SMLAWB(state_QS[i], state_QS[i + 1] - tmp1_QS, warping_Q16);
            state_QS[i] = tmp1_QS;
            corr_QC[i] += silk_RSHIFT64(silk_SMULL(tmp1_QS, state_QS[0]), 2 * QS - QC);
            /* Output of allpass section */
            tmp1_QS = silk_SMLAWB(state_QS[i + 1], state_QS[i + 2] - tmp2_QS, warping_Q16);
            state_QS[i + 1] = tmp2_QS;
            corr_QC[i + 1] += silk_RSHIFT64(silk_SMULL(tmp2_QS, state_QS[0]), 2 * QS - QC);
        }
        state_QS[order] = tmp1_QS;
        corr_QC[order] += silk_RSHIFT64(silk_SMULL(tmp1_QS, state_QS[0]), 2 * QS - QC);
    }

    lsh = silk_CLZ64(corr_QC[0]) - 35;
    lsh = silk_LIMIT(lsh, -12 - QC, 30 - QC);
    *scale = -(QC + lsh);
    assert(*scale >= -30 && *scale <= 12);
    if (lsh >= 0) {
        for (i = 0; i < order + 1; i++) {
            corr[i] = (int32_t)silk_CHECK_FIT32(silk_LSHIFT64(corr_QC[i], lsh));
        }
    } else {
        for (i = 0; i < order + 1; i++) {
            corr[i] = (int32_t)silk_CHECK_FIT32(silk_RSHIFT64(corr_QC[i], -lsh));
        }
    }
    assert(corr_QC[0] >= 0); /* If breaking, decrease QC*/
    free(state_QS);
    free(corr_QC);
}


