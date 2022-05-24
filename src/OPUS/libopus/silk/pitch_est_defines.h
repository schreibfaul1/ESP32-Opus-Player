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

#ifndef SILK_PE_DEFINES_H
#define SILK_PE_DEFINES_H

#include "SigProc_FIX.h"
#include "../celt/celt.h"
#include "silk.h"

/********************************************************/
/* Definitions for pitch estimator                      */
/********************************************************/



/* Tables for 20 ms frames */
extern const int8_t silk_CB_lags_stage2[ PE_MAX_NB_SUBFR ][ PE_NB_CBKS_STAGE2_EXT ];
extern const int8_t silk_CB_lags_stage3[ PE_MAX_NB_SUBFR ][ PE_NB_CBKS_STAGE3_MAX ];
extern const int8_t silk_Lag_range_stage3[ SILK_PE_MAX_COMPLEX + 1 ] [ PE_MAX_NB_SUBFR ][ 2 ];
extern const int8_t silk_nb_cbk_searchs_stage3[ SILK_PE_MAX_COMPLEX + 1 ];

/* Tables for 10 ms frames */
extern const int8_t silk_CB_lags_stage2_10_ms[ PE_MAX_NB_SUBFR >> 1][ 3 ];
extern const int8_t silk_CB_lags_stage3_10_ms[ PE_MAX_NB_SUBFR >> 1 ][ 12 ];
extern const int8_t silk_Lag_range_stage3_10_ms[ PE_MAX_NB_SUBFR >> 1 ][ 2 ];

#endif

