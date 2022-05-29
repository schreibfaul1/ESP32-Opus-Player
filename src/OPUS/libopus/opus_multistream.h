/* Copyright (c) 2011 Xiph.Org Foundation
   Written by Jean-Marc Valin */
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

/**
 * @file opus_multistream.h
 * @brief Opus reference implementation multistream API
 */

#ifndef OPUS_MULTISTREAM_H
#define OPUS_MULTISTREAM_H

#include "opus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define __opus_check_encstate_ptr(ptr) ((ptr) + ((ptr) - (OpusEncoder **)(ptr)))
#define __opus_check_decstate_ptr(ptr) ((ptr) + ((ptr) - (OpusDecoder **)(ptr)))
#define OPUS_MULTISTREAM_GET_DECODER_STATE_REQUEST 5122
#define OPUS_MULTISTREAM_GET_DECODER_STATE(x, y) \
    OPUS_MULTISTREAM_GET_DECODER_STATE_REQUEST, (int32_t)(x), __opus_check_decstate_ptr(y)
typedef struct OpusMSDecoder OpusMSDecoder;

int32_t opus_multistream_decoder_get_size(int streams, int coupled_streams);
OpusMSDecoder *opus_multistream_decoder_create(int32_t Fs, int channels, int streams, int coupled_streams,
                                               const unsigned char *mapping, int *error);
int opus_multistream_decoder_init(OpusMSDecoder *st, int32_t Fs, int channels, int streams, int coupled_streams,
                                  const unsigned char *mapping);
int opus_multistream_decode(OpusMSDecoder *st, const unsigned char *data, int32_t len, int16_t *pcm, int frame_size,
                            int decode_fec);
int opus_multistream_decoder_ctl(OpusMSDecoder *st, int request, ...);
void opus_multistream_decoder_destroy(OpusMSDecoder *st);


#ifdef __cplusplus
}
#endif

#endif /* OPUS_MULTISTREAM_H */
