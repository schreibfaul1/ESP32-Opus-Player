/* Copyright (c) 2010-2011 Xiph.Org Foundation, Skype Limited
   Written by Jean-Marc Valin and Koen Vos */
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
 * @file opus.h
 * @brief Opus reference implementation API
 */

#ifndef OPUS_H
#define OPUS_H

#include <stdint.h>
#include <stdarg.h> /* va_list */
#include <stddef.h> /* offsetof */

#ifdef __cplusplus
extern "C" {
#endif

#define OPUS_OK                0
#define OPUS_BAD_ARG_         -1
#define OPUS_BUFFER_TOO_SMALL -2
#define OPUS_INTERNAL_ERROR   -3
#define OPUS_INVALID_PACKET   -4
#define OPUS_UNIMPLEMENTED    -5
#define OPUS_INVALID_STATE    -6
#define OPUS_ALLOC_FAIL       -7

#define OPUS_GET_SAMPLE_RATE_REQUEST 4029
#define OPUS_BANDWIDTH_NARROWBAND 1101    /**< 4 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_MEDIUMBAND 1102    /**< 6 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_WIDEBAND 1103      /**< 8 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_SUPERWIDEBAND 1104 /**<12 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_FULLBAND 1105      /**<20 kHz bandpass @hideinitializer*/
#define OPUS_RESET_STATE 4028
#define MODE_CELT_ONLY 1002

typedef struct OpusDecoder OpusDecoder;
typedef struct CELTDecoder CELTDecoder;
typedef struct CELTMode CELTMode;
typedef enum { MAPPING_TYPE_NONE, MAPPING_TYPE_SURROUND, MAPPING_TYPE_AMBISONICS } MappingType;
typedef void (*downmix_func)(const void *, int32_t *, int32_t, int32_t, int32_t, int32_t, int32_t);
typedef void (*opus_copy_channel_in_func)(int16_t *dst, int32_t dst_stride, const void *src, int32_t src_stride,
                                          int32_t src_channel, int32_t frame_size, void *user_data);
typedef void (*opus_copy_channel_out_func)(void *dst, int32_t dst_stride, int32_t dst_channel, const int16_t *src,
                                           int32_t src_stride, int32_t frame_size, void *user_data);

typedef struct OpusMSDecoder {
}OpusMSDecoder_t;



int32_t opus_decoder_get_size(int32_t channels);
int32_t opus_decode(const uint8_t *data, int32_t len, int16_t *pcm, int32_t frame_size);
int32_t opus_decoder_ctl(int32_t request, int32_t* ap);
int32_t opus_packet_parse(const uint8_t *data, int32_t len, uint8_t *out_toc, const uint8_t *frames[48],
                      int16_t size[48], int32_t *payload_offset);
int32_t opus_packet_get_bandwidth(const uint8_t *data);
int32_t opus_packet_get_samples_per_frame(const uint8_t *data, int32_t Fs);
int32_t opus_packet_get_nb_channels(const uint8_t *data);
int32_t opus_packet_get_nb_frames(const uint8_t packet[], int32_t len);
int32_t opus_packet_get_nb_samples(const uint8_t packet[], int32_t len, int32_t Fs);
int32_t opus_decode_native(const uint8_t *data, int32_t len, int16_t *pcm, int32_t frame_size,
                       int32_t self_delimited, int32_t *packet_offset);

int32_t opus_packet_parse_impl(const uint8_t *data, int32_t len, int32_t self_delimited, uint8_t *out_toc,
                           const uint8_t *frames[48], int16_t size[48], int32_t *payload_offset,
                           int32_t *packet_offset);
int32_t opus_get_left_channel(int32_t stream_id, int32_t prev);
int32_t opus_get_right_channel(int32_t stream_id, int32_t prev);
int32_t opus_get_mono_channel(int32_t stream_id, int32_t prev);
bool opus_decoder_create(int32_t Fs, int32_t channels, const uint8_t *mapping, int32_t *error);
int32_t opus_multistream_decode(const uint8_t *data, int32_t len, int16_t *pcm, int32_t frame_size);
// void opus_multistream_decoder_destroy(OpusMSDecoder_t *st);
int32_t opus_multistream_packet_validate(const uint8_t *data, int32_t len, int32_t Fs);
int32_t opus_decode_frame(const uint8_t *data, int32_t len, int16_t *pcm, int32_t frame_size, int32_t decode_fec);

void opus_copy_channel_out_short(int16_t *dst, int32_t dst_channel, const int16_t *src, int32_t src_stride,
                                        int32_t frame_size);

#ifdef __cplusplus
}
#endif

#endif /* OPUS_H */
