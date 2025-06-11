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
#include "celt.h"
#include "silk.h"

#define VARDECL(type, var)
#define ALLOC(var, size, type) type var[size]
#define OPUS_OK                0
#define OPUS_BAD_ARG          -1
#define OPUS_BUFFER_TOO_SMALL -2
#define OPUS_INTERNAL_ERROR   -3
#define OPUS_INVALID_PACKET   -4
#define OPUS_UNIMPLEMENTED    -5
#define OPUS_INVALID_STATE    -6
#define OPUS_ALLOC_FAIL       -7

#define OPUS_MULTISTREAM_GET_DECODER_STATE_REQUEST 5122

#define OPUS_GNUC_PREREQ(_maj,_min) \
 ((__GNUC__<<16)+__GNUC_MINOR__>=((_maj)<<16)+(_min))

#define OPUS_SET_APPLICATION_REQUEST 4000
#define OPUS_GET_APPLICATION_REQUEST 4001
#define OPUS_SET_BITRATE_REQUEST 4002
#define OPUS_GET_BITRATE_REQUEST 4003
#define OPUS_SET_MAX_BANDWIDTH_REQUEST 4004
#define OPUS_GET_MAX_BANDWIDTH_REQUEST 4005
#define OPUS_SET_VBR_REQUEST 4006
#define OPUS_GET_VBR_REQUEST 4007
#define OPUS_SET_BANDWIDTH_REQUEST 4008
#define OPUS_GET_BANDWIDTH_REQUEST 4009
#define OPUS_SET_COMPLEXITY_REQUEST 4010
#define OPUS_GET_COMPLEXITY_REQUEST 4011
#define OPUS_SET_INBAND_FEC_REQUEST 4012
#define OPUS_GET_INBAND_FEC_REQUEST 4013
#define OPUS_SET_PACKET_LOSS_PERC_REQUEST 4014
#define OPUS_GET_PACKET_LOSS_PERC_REQUEST 4015
#define OPUS_SET_DTX_REQUEST 4016
#define OPUS_GET_DTX_REQUEST 4017
#define OPUS_SET_VBR_CONSTRAINT_REQUEST 4020
#define OPUS_GET_VBR_CONSTRAINT_REQUEST 4021
#define OPUS_SET_FORCE_CHANNELS_REQUEST 4022
#define OPUS_GET_FORCE_CHANNELS_REQUEST 4023
#define OPUS_SET_SIGNAL_REQUEST 4024
#define OPUS_GET_SIGNAL_REQUEST 4025
#define OPUS_GET_LOOKAHEAD_REQUEST 4027
#define OPUS_GET_SAMPLE_RATE_REQUEST 4029
#define OPUS_GET_FINAL_RANGE_REQUEST 4031
#define OPUS_GET_PITCH_REQUEST 4033
#define OPUS_SET_GAIN_REQUEST 4034
#define OPUS_GET_GAIN_REQUEST 4045 /* Should have been 4035 */
#define OPUS_SET_LSB_DEPTH_REQUEST 4036
#define OPUS_GET_LSB_DEPTH_REQUEST 4037
#define OPUS_GET_LAST_PACKET_DURATION_REQUEST 4039
#define OPUS_SET_EXPERT_FRAME_DURATION_REQUEST 4040
#define OPUS_GET_EXPERT_FRAME_DURATION_REQUEST 4041
#define OPUS_SET_PREDICTION_DISABLED_REQUEST 4042
#define OPUS_GET_PREDICTION_DISABLED_REQUEST 4043
#define OPUS_SET_PHASE_INVERSION_DISABLED_REQUEST 4046
#define OPUS_GET_PHASE_INVERSION_DISABLED_REQUEST 4047
#define OPUS_GET_IN_DTX_REQUEST 4049
#define OPUS_AUTO -1000     /**<Auto/default setting @hideinitializer*/
#define OPUS_BITRATE_MAX -1 /**<Maximum bitrate @hideinitializer*/
#define OPUS_APPLICATION_VOIP 2048
#define OPUS_APPLICATION_AUDIO 2049
#define OPUS_APPLICATION_RESTRICTED_LOWDELAY 2051
#define OPUS_SIGNAL_VOICE 3001            /**< Signal being encoded is voice */
#define OPUS_SIGNAL_MUSIC 3002            /**< Signal being encoded is music */
#define OPUS_BANDWIDTH_NARROWBAND 1101    /**< 4 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_MEDIUMBAND 1102    /**< 6 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_WIDEBAND 1103      /**< 8 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_SUPERWIDEBAND 1104 /**<12 kHz bandpass @hideinitializer*/
#define OPUS_BANDWIDTH_FULLBAND 1105      /**<20 kHz bandpass @hideinitializer*/
#define OPUS_RESET_STATE 4028
#define MODE_SILK_ONLY 1000
#define MODE_HYBRID 1001
#define MODE_CELT_ONLY 1002
#define OPUS_SET_VOICE_RATIO_REQUEST 11018
#define OPUS_GET_VOICE_RATIO_REQUEST 11019
#define OPUS_SET_FORCE_MODE_REQUEST 11002
#define OPUS_SET_FORCE_MODE(x) OPUS_SET_FORCE_MODE_REQUEST, (int32_t)(x)


typedef struct OpusDecoder OpusDecoder;
//typedef struct CELTDecoder CELTDecoder;
//typedef struct CELTMode CELTMode;
typedef enum { MAPPING_TYPE_NONE, MAPPING_TYPE_SURROUND, MAPPING_TYPE_AMBISONICS } MappingType;
typedef void (*downmix_func)(const void *, int32_t *, int, int, int, int, int);
typedef void (*opus_copy_channel_in_func)(int16_t *dst, int dst_stride, const void *src, int src_stride,
                                          int src_channel, int frame_size, void *user_data);
typedef void (*opus_copy_channel_out_func)(void *dst, int dst_stride, int dst_channel, const int16_t *src,
                                           int src_stride, int frame_size, void *user_data);

typedef struct OpusMSDecoder {
    int nb_channels;
    int nb_streams;
    int nb_coupled_streams;
    unsigned char mapping[256];
}OpusMSDecoder_t;



int opus_decoder_get_size(int channels);
int opus_decoder_init(OpusDecoder *st, int32_t Fs, int channels);
int opus_decode(OpusDecoder *st, uint8_t *data, int32_t len, int16_t *pcm, int frame_size);
int opus_decoder_ctl(OpusDecoder *st, int request, ...);
int opus_packet_parse(uint8_t *data, int32_t len, unsigned char *out_toc, uint8_t *frames[48], int16_t size[48], int *payload_offset);
int opus_packet_get_bandwidth(uint8_t *data);
int opus_packet_get_samples_per_frame(uint8_t *data, int32_t Fs);
int opus_packet_get_nb_channels(uint8_t *data);
int opus_packet_get_nb_frames(uint8_t packet[], int32_t len);
int opus_packet_get_nb_samples(uint8_t packet[], int32_t len, int32_t Fs);
int opus_decoder_get_nb_samples(const OpusDecoder *dec, uint8_t packet[], int32_t len);


//CELTMode *opus_custom_mode_create(int32_t Fs, int frame_size, int *error);
int celt_decoder_ctl(CELTDecoder_t *__restrict__ st, int request, ...);

int encode_size(int size, unsigned char *data);
int opus_decode_native(OpusDecoder *st, uint8_t *data, int32_t len, int16_t *pcm, int frame_size,
                       int self_delimited, int32_t *packet_offset);

/* Make sure everything is properly aligned. */
static inline int align(int i) {
    struct foo {
        char c;
        union {
            void *p;
            int32_t i;
            int32_t v;
        } u;
    };
    unsigned int alignment = offsetof(struct foo, u);

    /* Optimizing compilers should optimize div and multiply into and
       for all sensible alignment values. */
    return ((i + alignment - 1) / alignment) * alignment;
}

int opus_packet_parse_impl(uint8_t *data, int32_t len, int self_delimited, unsigned char *out_toc,
                           uint8_t *frames[48], int16_t size[48], int *payload_offset,
                           int32_t *packet_offset);
int opus_multistream_decode_native(OpusMSDecoder_t *st, uint8_t *data, int32_t len, void *pcm,
                                   opus_copy_channel_out_func copy_channel_out, int frame_size);
int opus_multistream_decoder_ctl_va_list(OpusMSDecoder_t *st, int request, va_list ap);
int validate_layout(OpusMSDecoder_t *layout);
int get_left_channel(OpusMSDecoder_t *layout, int stream_id, int prev);
int get_right_channel(OpusMSDecoder_t *layout, int stream_id, int prev);
int get_mono_channel(OpusMSDecoder_t *layout, int stream_id, int prev);
int32_t opus_multistream_decoder_get_size(int streams, int coupled_streams);
OpusMSDecoder_t *opus_multistream_decoder_create(int32_t Fs, int channels, int streams, int coupled_streams, const uint8_t *mapping, int *error);
int opus_multistream_decoder_init(OpusMSDecoder_t *st, int32_t Fs, int channels, int streams, int coupled_streams,
                                  const uint8_t *mapping);
int opus_multistream_decode(OpusMSDecoder_t *st, uint8_t *data, int32_t len, int16_t *pcm, int frame_size);
int opus_multistream_decoder_ctl(OpusMSDecoder_t *st, int request, ...);
void opus_multistream_decoder_destroy(OpusMSDecoder_t *st);















#endif /* OPUS_H */
