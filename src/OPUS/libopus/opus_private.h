/* Copyright (c) 2012 Xiph.Org Foundation
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


#ifndef OPUS_PRIVATE_H
#define OPUS_PRIVATE_H

#include "opus.h"
#include "celt/celt.h"

#include <stdarg.h> /* va_list */
#include <stddef.h> /* offsetof */



typedef struct ChannelLayout {
    int nb_channels;
    int nb_streams;
    int nb_coupled_streams;
    unsigned char mapping[256];
} ChannelLayout;

typedef enum { MAPPING_TYPE_NONE, MAPPING_TYPE_SURROUND, MAPPING_TYPE_AMBISONICS } MappingType;

// struct OpusMSEncoder {
//     ChannelLayout layout;
//     int arch;
//     int lfe_stream;
//     int application;
//     int variable_duration;
//     MappingType mapping_type;
//     int32_t bitrate_bps;
//     /* Encoder states go here */
//     /* then int32_t window_mem[channels*120]; */
//     /* then int32_t preemph_mem[channels]; */
// };

struct OpusMSDecoder {
    ChannelLayout layout;
    /* Decoder states go here */
};

//int opus_multistream_encoder_ctl_va_list(struct OpusMSEncoder *st, int request, va_list ap);
int opus_multistream_decoder_ctl_va_list(struct OpusMSDecoder *st, int request, va_list ap);

int validate_layout(const ChannelLayout *layout);
int get_left_channel(const ChannelLayout *layout, int stream_id, int prev);
int get_right_channel(const ChannelLayout *layout, int stream_id, int prev);
int get_mono_channel(const ChannelLayout *layout, int stream_id, int prev);

typedef void (*opus_copy_channel_in_func)(int16_t *dst, int dst_stride, const void *src, int src_stride,
                                          int src_channel, int frame_size, void *user_data);

typedef void (*opus_copy_channel_out_func)(void *dst, int dst_stride, int dst_channel, const int16_t *src,
                                           int src_stride, int frame_size, void *user_data);

#define MODE_SILK_ONLY 1000
#define MODE_HYBRID 1001
#define MODE_CELT_ONLY 1002

#define OPUS_SET_VOICE_RATIO_REQUEST 11018
#define OPUS_GET_VOICE_RATIO_REQUEST 11019

/** Configures the encoder's expected percentage of voice
 * opposed to music or other signals.
 *
 * @note This interface is currently more aspiration than actuality. It's
 * ultimately expected to bias an automatic signal classifier, but it currently
 * just shifts the static bitrate to mode mapping around a little bit.
 *
 * @param[in] x <tt>int</tt>:   Voice percentage in the range 0-100, inclusive.
 * @hideinitializer */
#define OPUS_SET_VOICE_RATIO(x) OPUS_SET_VOICE_RATIO_REQUEST, (int32_t)(x)
/** Gets the encoder's configured voice ratio value, @see OPUS_SET_VOICE_RATIO
 *
 * @param[out] x <tt>int*</tt>:  Voice percentage in the range 0-100, inclusive.
 * @hideinitializer */
#define OPUS_GET_VOICE_RATIO(x) OPUS_GET_VOICE_RATIO_REQUEST, (int32_t *)(x)

#define OPUS_SET_FORCE_MODE_REQUEST 11002
#define OPUS_SET_FORCE_MODE(x) OPUS_SET_FORCE_MODE_REQUEST, (int32_t)(x)

typedef void (*downmix_func)(const void *, int32_t *, int, int, int, int, int);
void downmix_float(const void *_x, int32_t *sub, int subframe, int offset, int c1, int c2, int C);
void downmix_int(const void *_x, int32_t *sub, int subframe, int offset, int c1, int c2, int C);
int is_digital_silence(const int16_t *pcm, int frame_size, int channels, int lsb_depth);

int encode_size(int size, unsigned char *data);

int32_t frame_size_select(int32_t frame_size, int variable_duration, int32_t Fs);

int opus_decode_native(OpusDecoder *st, const unsigned char *data, int32_t len, int16_t *pcm, int frame_size,
                       int decode_fec, int self_delimited, int32_t *packet_offset, int soft_clip);

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

int opus_packet_parse_impl(const unsigned char *data, int32_t len, int self_delimited, unsigned char *out_toc,
                           const unsigned char *frames[48], int16_t size[48], int *payload_offset,
                           int32_t *packet_offset);

int pad_frame(unsigned char *data, int32_t len, int32_t new_len);

int opus_multistream_decode_native(struct OpusMSDecoder *st, const unsigned char *data, int32_t len, void *pcm,
                                   opus_copy_channel_out_func copy_channel_out, int frame_size, int decode_fec,
                                   int soft_clip, void *user_data);

#endif /* OPUS_PRIVATE_H */
