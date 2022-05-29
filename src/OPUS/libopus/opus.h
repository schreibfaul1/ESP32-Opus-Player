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
#include "opus_defines.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct OpusDecoder OpusDecoder;
typedef struct CELTEncoder CELTEncoder;
typedef struct CELTDecoder CELTDecoder;
typedef struct CELTMode CELTMode;

int opus_decoder_get_size(int channels);
OpusDecoder *opus_decoder_create(int32_t Fs, int channels, int *error);
int opus_decoder_init(OpusDecoder *st, int32_t Fs, int channels);
int opus_decode(OpusDecoder *st, const unsigned char *data, int32_t len, int16_t *pcm, int frame_size, int decode_fec);
int opus_decode_float(OpusDecoder *st, const unsigned char *data, int32_t len, float *pcm, int frame_size,
                      int decode_fec);
int opus_decoder_ctl(OpusDecoder *st, int request, ...);
int opus_packet_parse(const unsigned char *data, int32_t len, unsigned char *out_toc, const unsigned char *frames[48],
                      int16_t size[48], int *payload_offset);
int opus_packet_get_bandwidth(const unsigned char *data);
int opus_packet_get_samples_per_frame(const unsigned char *data, int32_t Fs);
int opus_packet_get_nb_channels(const unsigned char *data);
int opus_packet_get_nb_frames(const unsigned char packet[], int32_t len);
int opus_packet_get_nb_samples(const unsigned char packet[], int32_t len, int32_t Fs);
int opus_decoder_get_nb_samples(const OpusDecoder *dec, const unsigned char packet[], int32_t len);


CELTMode *opus_custom_mode_create(int32_t Fs, int frame_size, int *error);
void opus_custom_mode_destroy(CELTMode *mode);
CELTEncoder *opus_custom_encoder_create(const CELTMode *mode, int channels, int *error);
void opus_custom_encoder_destroy(CELTEncoder *st);
int opus_custom_encode_float(CELTEncoder *st, const float *pcm, int frame_size, unsigned char *compressed,
                             int maxCompressedBytes);
int opus_custom_encode(CELTEncoder *st, const int16_t *pcm, int frame_size, unsigned char *compressed,
                       int maxCompressedBytes);
int celt_encoder_ctl(CELTEncoder *__restrict__ st, int request, ...);
CELTDecoder *opus_custom_decoder_create(const CELTMode *mode, int channels, int *error);
void opus_custom_decoder_destroy(CELTDecoder *st);
int opus_custom_decode_float(CELTDecoder *st, const unsigned char *data, int len, float *pcm, int frame_size);
int opus_custom_decode(CELTDecoder *st, const unsigned char *data, int len, int16_t *pcm, int frame_size);
int celt_decoder_ctl(CELTDecoder *__restrict__ st, int request, ...);


















#ifdef __cplusplus
}
#endif

#endif /* OPUS_H */
