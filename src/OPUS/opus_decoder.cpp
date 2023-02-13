/* Copyright (c) 2010 Xiph.Org Foundation, Skype Limited
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

#include "Arduino.h"
#include "celt.h"
#include <stdarg.h>
#include "opus_decoder.h"

const uint32_t CELT_SET_SIGNALLING_REQUEST      = 10016;
const uint32_t CELT_SET_END_BAND_REQUEST        = 10012;
const uint32_t CELT_GET_MODE_REQUEST            = 10015;

struct OpusDecoder {
    int32_t   celt_dec_offset;
    int32_t   channels;
    int32_t   Fs;          /** Sampling rate (at the API level) */
    int32_t   decode_gain;
    int32_t   arch;
    int32_t   stream_channels;
    int32_t   bandwidth;
    int32_t   mode;
    int32_t   prev_mode;
    int32_t   frame_size;
    int32_t   prev_redundancy;
    int32_t   last_packet_duration;
    uint32_t  rangeFinal;
} m_OpusDecoder;

    int32_t s_nb_channels;
    int32_t s_nb_streams;
    int32_t s_nb_coupled_streams;
    uint8_t s_mapping[2];

//----------------------------------------------------------------------------------------------------------------------
static int32_t opus_packet_get_mode(const uint8_t *data) {
    static int32_t oldmode;
    int32_t        mode;
    if(data[0] & 0x80) { mode = MODE_CELT_ONLY; }
    else
        log_e("no silk or hybrid mode supported");

    if(oldmode != mode) {
        oldmode = mode;
        if(mode == MODE_CELT_ONLY) log_i("opus mode is MODE_CELT_ONLY");
    }
    return mode;
}
//----------------------------------------------------------------------------------------------------------------------
int32_t opus_decode_frame(const uint8_t *inbuf, int32_t len, int16_t *outbuf, int32_t frame_size) {

    int32_t        i, celt_ret = 0;
    int32_t        audiosize;
    int32_t        mode;
    int32_t        F2_5, F5, F10, F20;

    F20 = m_OpusDecoder.Fs / 50;
    F10 = F20 >> 1;
    F5 = F10 >> 1;
    F2_5 = F5 >> 1;
    if(frame_size < F2_5) { return OPUS_BUFFER_TOO_SMALL; }
    /* Limit frame_size to avoid excessive stack allocations. */
    frame_size = min(frame_size, m_OpusDecoder.Fs / 25 * 3);
    /* Payloads of 1 (2 including ToC) or 0 trigger the PLC/DTX */
    if(len <= 1) {
        inbuf = NULL;
        /* In that case, don't conceal more than what the ToC says */
        frame_size = min(frame_size, m_OpusDecoder.frame_size);
    }
    if(inbuf != NULL) {
        audiosize = m_OpusDecoder.frame_size;
        mode = m_OpusDecoder.mode;
        ec_dec_init((uint8_t *)inbuf, len);
    }

    int32_t celt_frame_size = min(F20, frame_size);
    /* Make sure to discard any previous CELT state */
    if(mode != m_OpusDecoder.prev_mode && m_OpusDecoder.prev_mode > 0 && !m_OpusDecoder.prev_redundancy) celt_decoder_ctl(OPUS_RESET_STATE);
    /* Decode CELT */
    //log_i("decode %i, celt_frame_size %i", len, celt_frame_size);
    // static uint32_t ms = millis();
    // log_i("diff %u", millis()-ms);
    // ms = millis();
    celt_ret = celt_decode_with_ec(inbuf, len, outbuf, celt_frame_size);
    //log_i("celt_ret %i", celt_ret );


    if(len <= 1) m_OpusDecoder.rangeFinal = 0;
    m_OpusDecoder.prev_mode = mode;
    int x = celt_ret < 0 ? celt_ret : audiosize;
//     log_i("x %i", x);
    return x;
}
//----------------------------------------------------------------------------------------------------------------------
int32_t opus_decode_native(const uint8_t *data, int32_t len, int16_t *pcm, int32_t frame_size,
                           int32_t self_delimited, int32_t *packet_offset) {
    int32_t i, nb_samples;
    int32_t count, offset;
    uint8_t toc;
    int32_t packet_frame_size, packet_bandwidth, packet_mode, packet_stream_channels;
    /* 48 x 2.5 ms = 120 ms */
    int16_t size[48];
    //    VALIDATE_OPUS_DECODER(st);
    /* For FEC/PLC, frame_size has to be to have a multiple of 2.5 ms */
    if((len == 0 || data == NULL) && frame_size % (m_OpusDecoder.Fs / 400) != 0) { return OPUS_BAD_ARG_; }

    if(len == 0 || data == NULL) {
        int32_t pcm_count = 0;
        do {
            int32_t ret;
            log_i("opus_decode_frame");
            ret = opus_decode_frame(NULL, 0, pcm + pcm_count * m_OpusDecoder.channels, frame_size - pcm_count);
            if(ret < 0) { return ret; }
            pcm_count += ret;
        } while(pcm_count < frame_size);
        assert(pcm_count == frame_size);
        //    if (OPUS_CHECK_ARRAY(pcm, pcm_count * m_OpusDecoder.hannels)) OPUS_PRINT_INT(pcm_count);
        m_OpusDecoder.last_packet_duration = pcm_count;
        return pcm_count;
    }
    else if(len < 0) { return OPUS_BAD_ARG_; }
    packet_mode = opus_packet_get_mode(data);
    packet_bandwidth = opus_packet_get_bandwidth(data);
    packet_frame_size = opus_packet_get_samples_per_frame(data, m_OpusDecoder.Fs);
    packet_stream_channels = opus_packet_get_nb_channels(data);

    count = opus_packet_parse_impl(data, len, self_delimited, &toc, NULL, size, &offset, packet_offset);
    if(count < 0) { return count; }
    data += offset;

    if(count * packet_frame_size > frame_size) { return OPUS_BUFFER_TOO_SMALL; }

    /* Update the state as the last step to avoid updating it on an invalid packet */
    m_OpusDecoder.mode = packet_mode;
    m_OpusDecoder.bandwidth = packet_bandwidth;
    m_OpusDecoder.frame_size = packet_frame_size;
    m_OpusDecoder.stream_channels = packet_stream_channels;

    nb_samples = 0;
    for(i = 0; i < count; i++) {
        int32_t ret;
        ret = opus_decode_frame(data, size[i], pcm + nb_samples * m_OpusDecoder.channels, frame_size - nb_samples);
        if(ret < 0) { return ret; }

        assert(ret == packet_frame_size);
        data += size[i];
        nb_samples += ret;
    }
    m_OpusDecoder.last_packet_duration = nb_samples;
    //    if (OPUS_CHECK_ARRAY(pcm, nb_samples * m_OpusDecoder.channels)) OPUS_PRINT_INT(nb_samples);
    return nb_samples;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_decode(const uint8_t *data, int32_t len, int16_t *pcm, int32_t frame_size) {
    if (frame_size <= 0) return OPUS_BAD_ARG_;
    int32_t ret = opus_decode_native(data, len, pcm, frame_size, 0, 0);
    log_i("bytes_decoded = %i", ret);
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_decoder_ctl(int32_t request, int32_t* ap) {

    int32_t ret = OPUS_OK;

    switch (request) {
        case OPUS_RESET_STATE: {
            m_OpusDecoder.stream_channels = 0;
            m_OpusDecoder.bandwidth = 0;
            m_OpusDecoder.mode = 0;
            m_OpusDecoder.prev_mode = 0;
            m_OpusDecoder.frame_size = 0;
            m_OpusDecoder.prev_redundancy = 0;
            m_OpusDecoder.last_packet_duration = 0;
            celt_decoder_ctl(OPUS_RESET_STATE);
            m_OpusDecoder.stream_channels = m_OpusDecoder.channels;
            m_OpusDecoder.frame_size = m_OpusDecoder.Fs / 400;
        } break;
        case OPUS_GET_SAMPLE_RATE_REQUEST: {
            *ap = m_OpusDecoder.Fs;
        } break;
        default:
            /*fprintf(stderr, "unknown opus_decoder_ctl() request: %d", request);*/
            ret = OPUS_UNIMPLEMENTED;
            break;
    }

    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_packet_get_bandwidth(const uint8_t *data) {
    int32_t bandwidth;
    if (data[0] & 0x80) {
        bandwidth = OPUS_BANDWIDTH_MEDIUMBAND + ((data[0] >> 5) & 0x3);
        if (bandwidth == OPUS_BANDWIDTH_MEDIUMBAND) bandwidth = OPUS_BANDWIDTH_NARROWBAND;
    } else if ((data[0] & 0x60) == 0x60) {
        bandwidth = (data[0] & 0x10) ? OPUS_BANDWIDTH_FULLBAND : OPUS_BANDWIDTH_SUPERWIDEBAND;
    } else {
        bandwidth = OPUS_BANDWIDTH_NARROWBAND + ((data[0] >> 5) & 0x3);
    }
    return bandwidth;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_packet_get_nb_channels(const uint8_t *data) { return (data[0] & 0x4) ? 2 : 1; }
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_packet_get_nb_frames(const uint8_t packet[], int32_t len) {
    int32_t count;
    if (len < 1) return OPUS_BAD_ARG_;
    count = packet[0] & 0x3;
    if (count == 0)
        return 1;
    else if (count != 3)
        return 2;
    else if (len < 2)
        return OPUS_INVALID_PACKET;
    else
        return packet[1] & 0x3F;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_packet_get_nb_samples(const uint8_t packet[], int32_t len, int32_t Fs) {
    int32_t samples;
    int32_t count = opus_packet_get_nb_frames(packet, len);

    if (count < 0) return count;

    samples = count * opus_packet_get_samples_per_frame(packet, Fs);
    /* Can't have more than 120 ms */
    if (samples * 25 > Fs * 3)
        return OPUS_INVALID_PACKET;
    else
        return samples;
}
//----------------------------------------------------------------------------------------------------------------------

static int32_t opus_parse_size(const uint8_t *data, int32_t len, int16_t *size) {
    if (len < 1) {
        *size = -1;
        return -1;
    } else if (data[0] < 252) {
        *size = data[0];
        return 1;
    } else if (len < 2) {
        *size = -1;
        return -1;
    } else {
        *size = 4 * data[1] + data[0];
        return 2;
    }
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_packet_get_samples_per_frame(const uint8_t *data, int32_t Fs) {
    int32_t audiosize;
    if (data[0] & 0x80) {
        audiosize = ((data[0] >> 3) & 0x3);
        audiosize = (Fs << audiosize) / 400;
    } else if ((data[0] & 0x60) == 0x60) {
        audiosize = (data[0] & 0x08) ? Fs / 50 : Fs / 100;
    } else {
        audiosize = ((data[0] >> 3) & 0x3);
        if (audiosize == 3)
            audiosize = Fs * 60 / 1000;
        else
            audiosize = (Fs << audiosize) / 100;
    }
    return audiosize;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_packet_parse_impl(const uint8_t *data, int32_t len, int32_t self_delimited, uint8_t *out_toc,
                           const uint8_t *frames[48], int16_t size[48], int32_t *payload_offset,
                           int32_t *packet_offset) {
    int32_t i, bytes;
    int32_t count;
    int32_t cbr;
    uint8_t ch, toc;
    int32_t framesize;
    int32_t last_size;
    int32_t pad = 0;
    const uint8_t *data0 = data;

    if (size == NULL || len < 0) return OPUS_BAD_ARG_;
    if (len == 0) return OPUS_INVALID_PACKET;

    framesize = opus_packet_get_samples_per_frame(data, 48000);

    cbr = 0;
    toc = *data++;
    len--;
    last_size = len;
    switch (toc & 0x3) {
        /* One frame */
        case 0:
            count = 1;
            break;
        /* Two CBR frames */
        case 1:
            count = 2;
            cbr = 1;
            if (!self_delimited) {
                if (len & 0x1) return OPUS_INVALID_PACKET;
                last_size = len / 2;
                /* If last_size doesn't fit in size[0], we'll catch it later */
                size[0] = (int16_t)last_size;
            }
            break;
        /* Two VBR frames */
        case 2:
            count = 2;
            bytes = opus_parse_size(data, len, size);
            len -= bytes;
            if (size[0] < 0 || size[0] > len) return OPUS_INVALID_PACKET;
            data += bytes;
            last_size = len - size[0];
            break;
        /* Multiple CBR/VBR frames (from 0 to 120 ms) */
        default: /*case 3:*/
            if (len < 1) return OPUS_INVALID_PACKET;
            /* Number of frames encoded in bits 0 to 5 */
            ch = *data++;
            count = ch & 0x3F;
            if (count <= 0 || framesize * (int32_t)count > 5760) return OPUS_INVALID_PACKET;
            len--;
            /* Padding flag is bit 6 */
            if (ch & 0x40) {
                int32_t p;
                do {
                    int32_t tmp;
                    if (len <= 0) return OPUS_INVALID_PACKET;
                    p = *data++;
                    len--;
                    tmp = p == 255 ? 254 : p;
                    len -= tmp;
                    pad += tmp;
                } while (p == 255);
            }
            if (len < 0) return OPUS_INVALID_PACKET;
            /* VBR flag is bit 7 */
            cbr = !(ch & 0x80);
            if (!cbr) {
                /* VBR case */
                last_size = len;
                for (i = 0; i < count - 1; i++) {
                    bytes = opus_parse_size(data, len, size + i);
                    len -= bytes;
                    if (size[i] < 0 || size[i] > len) return OPUS_INVALID_PACKET;
                    data += bytes;
                    last_size -= bytes + size[i];
                }
                if (last_size < 0) return OPUS_INVALID_PACKET;
            } else if (!self_delimited) {
                /* CBR case */
                last_size = len / count;
                if (last_size * count != len) return OPUS_INVALID_PACKET;
                for (i = 0; i < count - 1; i++) size[i] = (int16_t)last_size;
            }
            break;
    }
    /* Self-delimited framing has an extra size for the last frame. */
    if (self_delimited) {
        bytes = opus_parse_size(data, len, size + count - 1);
        len -= bytes;
        if (size[count - 1] < 0 || size[count - 1] > len) return OPUS_INVALID_PACKET;
        data += bytes;
        /* For CBR packets, apply the size to all the frames. */
        if (cbr) {
            if (size[count - 1] * count > len) return OPUS_INVALID_PACKET;
            for (i = 0; i < count - 1; i++) size[i] = size[count - 1];
        } else if (bytes + size[count - 1] > last_size)
            return OPUS_INVALID_PACKET;
    } else {
        /* Because it's not encoded explicitly, it's possible the size of the
           last packet (or all the packets, for the CBR case) is larger than
           1275. Reject them here.*/
        if (last_size > 1275) return OPUS_INVALID_PACKET;
        size[count - 1] = (int16_t)last_size;
    }

    if (payload_offset) *payload_offset = (int32_t)(data - data0);

    for (i = 0; i < count; i++) {
        if (frames) frames[i] = data;
        data += size[i];
    }

    if (packet_offset) *packet_offset = pad + (int32_t)(data - data0);

    if (out_toc) *out_toc = toc;

    return count;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_packet_parse(const uint8_t *data, int32_t len, uint8_t *out_toc, const uint8_t *frames[48],
                      int16_t size[48], int32_t *payload_offset) {
    return opus_packet_parse_impl(data, len, 0, out_toc, frames, size, payload_offset, NULL);
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_get_left_channel(int32_t stream_id, int32_t prev) {
    int32_t i;
    i = (prev < 0) ? 0 : prev + 1;
    for (; i < s_nb_channels; i++) {
        if (s_mapping[i] == stream_id * 2) return i;
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_get_right_channel(int32_t stream_id, int32_t prev) {
    int32_t i;
    i = (prev < 0) ? 0 : prev + 1;
    for (; i < s_nb_channels; i++) {
        if (s_mapping[i] == stream_id * 2 + 1) return i;
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_get_mono_channel(int32_t stream_id, int32_t prev) {
    int32_t i;
    i = (prev < 0) ? 0 : prev + 1;
    for (; i < s_nb_channels; i++) {
        if (s_mapping[i] == stream_id + s_nb_coupled_streams) return i;
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------

bool opus_decoder_create(int32_t Fs, int32_t channels, const uint8_t *mapping, int32_t *error) {

    if(Fs != 48000 || (channels != 1 && channels != 2)) return false; // OPUS_BAD_ARG_;

    int32_t ret;
    if (channels > 2){
        if (error) *error = OPUS_BAD_ARG_;
        return false;
    }

    if(!CELTDecoder_AllocateBuffers()){if (error) *error = OPUS_ALLOC_FAIL; return false;}

    s_nb_channels = channels;
    s_nb_streams = 1;
    s_nb_coupled_streams = 1;

    for (int i = 0; i < s_nb_channels; i++) s_mapping[i] = mapping[i];

    CELTDecoder_ClearBuffer();

    m_OpusDecoder.stream_channels = m_OpusDecoder.channels = channels;
    m_OpusDecoder.Fs = Fs;

    ret = celt_decoder_init(channels);
    if (ret != OPUS_OK)
        return ret;

    celt_decoder_ctl(CELT_SET_SIGNALLING_REQUEST, 0);

    m_OpusDecoder.prev_mode = 0;
    m_OpusDecoder.frame_size = Fs / 400;
    m_OpusDecoder.arch = 0;

//---------------------------------------------------------------------------------

    if (ret != OPUS_OK) {
        CELTDecoder_FreeBuffers();
        return false;
    }
    if (error) *error = ret;
    if (ret != OPUS_OK) {
        CELTDecoder_FreeBuffers();
        return false;
    }
    return true;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_multistream_packet_validate(const uint8_t *data, int32_t len, int32_t Fs) {

    int32_t s;
    int32_t count;
    uint8_t toc;
    int16_t size[48];
    int32_t samples = 0;
    int32_t packet_offset;

    for (s = 0; s < s_nb_streams; s++) {
        int32_t tmp_samples;
        if (len <= 0) return OPUS_INVALID_PACKET;
        count = opus_packet_parse_impl(data, len, s != s_nb_streams - 1, &toc, NULL, size, NULL, &packet_offset);
        if (count < 0) return count;
        tmp_samples = opus_packet_get_nb_samples(data, packet_offset, Fs);
        if (s != 0 && samples != tmp_samples) return OPUS_INVALID_PACKET;
        samples = tmp_samples;
        data += packet_offset;
        len -= packet_offset;
    }
    return samples;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t opus_multistream_decode(const uint8_t *data, int32_t len, int16_t *pcm, int32_t frame_size) {
    int32_t Fs;
    int32_t s, c;
    int32_t do_plc = 0;
    if (frame_size <= 0) {
        return OPUS_BAD_ARG_;
    }
    /* Limit frame_size to avoid excessive stack allocations. */
    Fs =  m_OpusDecoder.Fs;
    frame_size = min(frame_size, Fs / 25 * 3);
    int16_t buf[2 * frame_size];

    if (len == 0) do_plc = 1;
    if (len < 0) {
        return OPUS_BAD_ARG_;
    }
    if (!do_plc && len < 2 * s_nb_streams - 1) {
        return OPUS_INVALID_PACKET;
    }
    if (!do_plc) {
        int32_t ret = opus_multistream_packet_validate(data, len, Fs);
        if (ret < 0) {
            return ret;
        } else if (ret > frame_size) {
            return OPUS_BUFFER_TOO_SMALL;
        }
    }
    for (s = 0; s < s_nb_streams; s++) {
        int32_t packet_offset;
        int32_t ret;

        if (!do_plc && len <= 0) {
            return OPUS_INTERNAL_ERROR;
        }
        packet_offset = 0;
        ret = opus_decode_native(data, len, buf, frame_size, s != s_nb_streams - 1, &packet_offset);
        data += packet_offset;
        len -= packet_offset;
        if (ret <= 0) {
            return ret;
        }
        frame_size = ret;
        if (s < s_nb_coupled_streams) {
            int32_t chan, prev;
            prev = -1;
            /* Copy "left" audio to the channel(s) where it belongs */
            while ((chan = opus_get_left_channel(s, prev)) != -1) {
                opus_copy_channel_out_short(pcm, chan, buf, 2, frame_size);
                prev = chan;
            }
            prev = -1;
            /* Copy "right" audio to the channel(s) where it belongs */
            while ((chan = opus_get_right_channel(s, prev)) != -1) {
                opus_copy_channel_out_short(pcm, chan, buf + 1, 2, frame_size);
                prev = chan;
            }
        } else {
            int32_t chan, prev;
            prev = -1;
            /* Copy audio to the channel(s) where it belongs */
            while ((chan = opus_get_mono_channel(s, prev)) != -1) {
                opus_copy_channel_out_short(pcm, chan, buf, 1, frame_size);
                prev = chan;
            }
        }
    }
    /* Handle muted channels */
    for (c = 0; c < s_nb_channels; c++) {
        if (s_mapping[c] == 255) {
            opus_copy_channel_out_short(pcm, c, NULL, 0, frame_size);
        }
    }

    return frame_size;
}
//----------------------------------------------------------------------------------------------------------------------

void opus_copy_channel_out_short(int16_t *dst, int32_t dst_channel, const int16_t *src, int32_t src_stride, int32_t frame_size) {
    int32_t i;
    if (src != NULL) {
        for (i = 0; i < frame_size; i++) dst[i * s_nb_channels + dst_channel] = src[i * src_stride];

    } else {
        for (i = 0; i < frame_size; i++) dst[i * s_nb_channels + dst_channel] = 0;
    }
}
//----------------------------------------------------------------------------------------------------------------------
