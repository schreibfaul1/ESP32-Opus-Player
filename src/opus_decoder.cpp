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
#include "opus_decoder.h"



const uint32_t CELT_SET_SIGNALLING_REQUEST      = 10016;
const uint32_t CELT_SET_END_BAND_REQUEST        = 10012;
const uint32_t CELT_SET_START_BAND_REQUEST      = 10010;
const uint32_t CELT_GET_MODE_REQUEST            = 10015;

extern silk_ptr_obj<silk_DecControlStruct_t>    s_silk_DecControlStruct;

struct OpusDecoder {
   int          celt_dec_offset;
   int          silk_dec_offset;
   int          channels;
   int32_t      Fs;          /** Sampling rate (at the API level) */
//   silk_DecControlStruct DecControl;
   int          decode_gain;
   int          arch;

   /* Everything beyond this point gets cleared on a reset */
#define OPUS_DECODER_RESET_START stream_channels
   int          stream_channels;
   int          bandwidth;
   int          mode;
   int          prev_mode;
   int          frame_size;
   int          prev_redundancy;
   int          last_packet_duration;
   uint32_t     rangeFinal;
};


#define VALIDATE_OPUS_DECODER(st)

//----------------------------------------------------------------------------------------------------------------------

int opus_decoder_get_size(int channels)
{
   int32_t silkDecSizeBytes, celtDecSizeBytes;
   int ret;
   if (channels<1 || channels > 2)
      return 0;
   ret = silk_Get_Decoder_Size( &silkDecSizeBytes );
   if(ret)
      return 0;
   silkDecSizeBytes = align(silkDecSizeBytes);
   celtDecSizeBytes = celt_decoder_get_size(channels);
   size_t sizeOfDecoders = align(sizeof(OpusDecoder))+silkDecSizeBytes+celtDecSizeBytes;
   // log_i("opus_decoder_get_size %d", sizeOfDecoders);
   return sizeOfDecoders;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_decoder_init(OpusDecoder *st, int32_t Fs, int channels)
{
   void *silk_dec;
   CELTDecoder_t *celt_dec;
   int32_t ret, silkDecSizeBytes;

   if ((Fs!=48000&&Fs!=24000&&Fs!=16000&&Fs!=12000&&Fs!=8000)
    || (channels!=1&&channels!=2))
      return OPUS_BAD_ARG;

   OPUS_CLEAR((char*)st, opus_decoder_get_size(channels));
   /* Initialize SILK decoder */
   ret = silk_Get_Decoder_Size(&silkDecSizeBytes);
   if (ret)
      return OPUS_INTERNAL_ERROR;

   silkDecSizeBytes = align(silkDecSizeBytes);
   st->silk_dec_offset = align(sizeof(OpusDecoder));
   st->celt_dec_offset = st->silk_dec_offset+silkDecSizeBytes;
   silk_dec = (char*)st+st->silk_dec_offset;
   celt_dec = (CELTDecoder_t*)((char*)st+st->celt_dec_offset);
   st->stream_channels = st->channels = channels;

   st->Fs = Fs;
   s_silk_DecControlStruct->API_sampleRate = st->Fs;
   s_silk_DecControlStruct->nChannelsAPI      = st->channels;

   /* Reset decoder */
   ret = silk_InitDecoder();
   if(ret)return OPUS_INTERNAL_ERROR;

   /* Initialize CELT decoder */
   ret = celt_decoder_init(celt_dec, Fs, channels);
   if(ret!=OPUS_OK)return OPUS_INTERNAL_ERROR;

   celt_decoder_ctl(celt_dec, (int32_t)CELT_SET_SIGNALLING_REQUEST, 0);

   st->prev_mode = 0;
   st->frame_size = Fs/400;
   st->arch = opus_select_arch();
   return OPUS_OK;
}

//----------------------------------------------------------------------------------------------------------------------

static void smooth_fade(const int16_t *in1, const int16_t *in2, int16_t *out, int overlap, int channels,
                        const int16_t *window, int32_t Fs) {
    int i, c;
    int inc = 48000 / Fs;
    for (c = 0; c < channels; c++) {
        for (i = 0; i < overlap; i++) {
            int16_t w = MULT16_16_Q15(window[i * inc], window[i * inc]);
            out[i * channels + c] =
                SHR32(MAC16_16(MULT16_16(w, in2[i * channels + c]), 32767 - w, in1[i * channels + c]), 15);
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------
static int opus_packet_get_mode(uint8_t *data) {
    static int oldmode;
	int mode;
    if (data[0] & 0x80) {
        mode = MODE_CELT_ONLY;
    } else if ((data[0] & 0x60) == 0x60) {
        mode = MODE_HYBRID;
    } else {
        mode = MODE_SILK_ONLY;
    }
	if(oldmode != mode){
		oldmode = mode;
		if(mode == MODE_CELT_ONLY) log_i("opus mode is MODE_CELT_ONLY");
		if(mode == MODE_SILK_ONLY) log_i("opus mode is MODE_SILK_ONLY");
		if(mode == MODE_HYBRID   ) log_i("opus mode is MODE_HYBRID   ");
	}
    return mode;
}
//----------------------------------------------------------------------------------------------------------------------
static int opus_decode_frame(OpusDecoder *st, uint8_t *inbuf, int32_t packetLen, int16_t *outbuf, int16_t samplesPerFrame) {
if(!inbuf)log_e("Inbuf is null");
    void *silk_dec;
    CELTDecoder_t *celt_dec;
    int i, silk_ret = 0, celt_ret = 0;
    ec_dec dec;
    int32_t silk_frame_size;
    int pcm_silk_size;
    VARDECL(int16_t, pcm_silk);

    uint16_t audiosize = 960;
    uint8_t payloadSize_ms = 0;
    uint16_t mode = st->mode; // MODE_CELT_ONLY or MODE_HYBRID or MODE_SILK_ONLY
    int bandwidth = st->bandwidth;
    uint8_t start_band;
    uint16_t internalSampleRate = 0;
    uint8_t channels = st->stream_channels;

    uint16_t F20 = 48000 / 50;  // 960
    uint16_t F10 = F20 >> 1;    // 480
    uint16_t F5 = F10 >> 1;     // 240
    uint16_t F2_5 = F5 >> 1;    // 120

    silk_dec = (char *)st + st->silk_dec_offset;
    celt_dec = (CELTDecoder_t *)((char *)st + st->celt_dec_offset);


//____________________________________________________________________________________________________________________________--


    ec_dec_init(&dec, inbuf, packetLen);

    /* Don't allocate any memory when in CELT-only mode */
    pcm_silk_size = (mode != MODE_CELT_ONLY) ? max(F10, audiosize) * channels : ALLOC_NONE;
    ALLOC(pcm_silk, pcm_silk_size, int16_t);

    /* SILK processing */
    if (mode != MODE_CELT_ONLY) {
        int lost_flag, decoded_samples;
        int16_t *pcm_ptr;
        pcm_ptr = pcm_silk;

        if (st->prev_mode == MODE_CELT_ONLY) silk_InitDecoder();

        /* The SILK PLC cannot produce frames of less than 10 ms */
        payloadSize_ms = max(10, 1000 * audiosize / 48000);

            s_silk_DecControlStruct->nChannelsInternal = channels;
            if (mode == MODE_SILK_ONLY) {
                if (bandwidth == OPUS_BANDWIDTH_NARROWBAND) {
                    internalSampleRate = 8000;
                } else if (bandwidth == OPUS_BANDWIDTH_MEDIUMBAND) {
                    internalSampleRate = 12000;
                } else if (bandwidth == OPUS_BANDWIDTH_WIDEBAND) {
                    internalSampleRate = 16000;
                } else {
                    internalSampleRate = 16000;
                }
            } else { /* Hybrid mode */
                internalSampleRate = 16000;
            }

        decoded_samples = 0;
        silk_setRawParams(channels, 2, payloadSize_ms, internalSampleRate, 48000);
        do {
            /* Call SILK decoder */
            int first_frame = decoded_samples == 0;
            silk_ret = silk_Decode(0, first_frame, pcm_ptr, &silk_frame_size);
            if (silk_ret) {
                    return OPUS_INTERNAL_ERROR;
            }
            pcm_ptr += silk_frame_size * channels;
            decoded_samples += silk_frame_size;
        } while (decoded_samples < audiosize);
    }

    start_band = 0;
    if (mode != MODE_CELT_ONLY && ec_tell(&dec) + 17 + 20 * (st->mode == MODE_HYBRID) <= 8 * packetLen) {
        /* Check if we have a redundant 0-8 kHz band */
        if (mode == MODE_HYBRID) ec_dec_bit_logp(12);
    }
    if (mode != MODE_CELT_ONLY) start_band = 17;

    if (bandwidth) {
        int endband = 21;

        switch (bandwidth) {
            case OPUS_BANDWIDTH_NARROWBAND:
                endband = 13;
                break;
            case OPUS_BANDWIDTH_MEDIUMBAND:
            case OPUS_BANDWIDTH_WIDEBAND:
                endband = 17;
                break;
            case OPUS_BANDWIDTH_SUPERWIDEBAND:
                endband = 19;
                break;
            case OPUS_BANDWIDTH_FULLBAND:
                endband = 21;
                break;
            default:
                break;
        }
        const uint32_t CELT_SET_CHANNELS_REQUEST        = 10008;
        silk_setRawParams(channels, 2, payloadSize_ms, internalSampleRate, 48000);
        celt_decoder_ctl(celt_dec, (int32_t)CELT_SET_END_BAND_REQUEST,(endband));
        celt_decoder_ctl(celt_dec, (int32_t)CELT_SET_CHANNELS_REQUEST,(channels));
    }

    /* MUST be after PLC */
    celt_decoder_ctl(celt_dec, (int32_t)CELT_SET_START_BAND_REQUEST, start_band);

    if (mode != MODE_SILK_ONLY) {
        int celt_frame_size = min(F20, audiosize);
        /* Make sure to discard any previous CELT state */
        if (mode != st->prev_mode && st->prev_mode > 0  /*&& !st->prev_redundancy */)
            celt_decoder_ctl(celt_dec, (int32_t)OPUS_RESET_STATE);
        /* Decode CELT */
        celt_ret = celt_decode_with_ec(celt_dec, inbuf, packetLen, outbuf, celt_frame_size, &dec, 0);
    } else {
        unsigned char silence[2] = {0xFF, 0xFF};
        for (i = 0; i < audiosize * channels; i++) outbuf[i] = 0;
        /* For hybrid -> SILK transitions, we let the CELT MDCT
           do a fade-out by decoding a silence frame */
        if (st->prev_mode == MODE_HYBRID) {
            celt_decoder_ctl(celt_dec, (int32_t)CELT_SET_START_BAND_REQUEST, 0);
            celt_decode_with_ec(celt_dec, silence, 2, outbuf, F2_5, NULL, 0);
        }
    }

    if (mode != MODE_CELT_ONLY) {
        for (i = 0; i < audiosize * channels; i++) outbuf[i] = SAT16(ADD32(outbuf[i], pcm_silk[i]));
    }

    {
        const CELTMode_t *celt_mode;
        silk_setRawParams(channels, 2, payloadSize_ms, internalSampleRate, 48000);
        celt_decoder_ctl(celt_dec, (int32_t)CELT_GET_MODE_REQUEST,(&celt_mode));
    }

    // if (packetLen <= 1)
    //     st->rangeFinal = 0;
    // else
    //     st->rangeFinal = dec.rng;

    st->prev_mode = mode;

    return celt_ret < 0 ? celt_ret : audiosize;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_decode_native(OpusDecoder *st, uint8_t *data, int32_t len, int16_t *pcm, int frame_size,
                       int self_delimited, int32_t *packet_offset) {
    int i, nb_samples;
    int count, offset;
    unsigned char toc;
    int packet_frame_size, packet_bandwidth, packet_mode, packet_stream_channels;
    /* 48 x 2.5 ms = 120 ms */
    int16_t size[48];
    VALIDATE_OPUS_DECODER(st);
    /* For FEC/PLC, frame_size has to be to have a multiple of 2.5 ms */
    if ((len == 0 || data == NULL) && frame_size % (st->Fs / 400) != 0) {
        return OPUS_BAD_ARG;
    }

    if (len == 0 || data == NULL) {
        int pcm_count = 0;
        do {
            int ret;
            log_i("opus_decode_frame");
            ret = opus_decode_frame(st, NULL, 0, pcm + pcm_count * st->channels, frame_size - pcm_count);
            if (ret < 0) {
                return ret;
            }
            pcm_count += ret;
        } while (pcm_count < frame_size);
        assert(pcm_count == frame_size);
    //    if (OPUS_CHECK_ARRAY(pcm, pcm_count * st->channels)) OPUS_PRINT_INT(pcm_count);
        st->last_packet_duration = pcm_count;
        return pcm_count;
    } else if (len < 0) {
        return OPUS_BAD_ARG;
    }
    packet_mode = opus_packet_get_mode(data);
    packet_bandwidth = opus_packet_get_bandwidth(data);
    packet_frame_size = opus_packet_get_samples_per_frame(data, st->Fs);
    packet_stream_channels = opus_packet_get_nb_channels(data);

    count = opus_packet_parse_impl(data, len, self_delimited, &toc, NULL, size, &offset, packet_offset);
    if (count < 0) {
        return count;
    }
    data += offset;

    if (count * packet_frame_size > frame_size) {
        return OPUS_BUFFER_TOO_SMALL;
    }

    /* Update the state as the last step to avoid updating it on an invalid packet */
    st->mode = packet_mode;
    st->bandwidth = packet_bandwidth;
    st->frame_size = packet_frame_size;
    st->stream_channels = packet_stream_channels;

    nb_samples = 0;
    for (i = 0; i < count; i++) {
        int ret;
        ret = opus_decode_frame(st, data, size[i], pcm + nb_samples * st->channels, frame_size - nb_samples);
        if (ret < 0) {
            return ret;
        }

        assert(ret == packet_frame_size);
        data += size[i];
        nb_samples += ret;
    }
    st->last_packet_duration = nb_samples;
//    if (OPUS_CHECK_ARRAY(pcm, nb_samples * st->channels)) OPUS_PRINT_INT(nb_samples);
    return nb_samples;
}
//----------------------------------------------------------------------------------------------------------------------

int opus_decode(OpusDecoder *st, uint8_t *data, int32_t len, int16_t *pcm, int frame_size) {
    if (frame_size <= 0) return OPUS_BAD_ARG;
    int ret = opus_decode_native(st, data, len, pcm, frame_size, 0, 0);
    log_i("bytes_decoded = %i", ret);
    log_e("opus decode len=%i, frame_size=%i, decode_fec=%i", len, frame_size);

    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

int opus_decoder_ctl(OpusDecoder *st, int request, ...) {
    int ret = OPUS_OK;
    va_list ap;
    void *silk_dec;
    CELTDecoder_t *celt_dec;

    silk_dec = (char *)st + st->silk_dec_offset;
    celt_dec = (CELTDecoder_t *)((char *)st + st->celt_dec_offset);

    va_start(ap, request);

    switch (request) {
        case OPUS_GET_BANDWIDTH_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            *value = st->bandwidth;
        } break;
        case OPUS_GET_FINAL_RANGE_REQUEST: {
            uint32_t *value = va_arg(ap, uint32_t *);
            if (!value) {
                goto bad_arg;
            }
            *value = st->rangeFinal;
        } break;
        case OPUS_RESET_STATE: {
            OPUS_CLEAR((char *)&st->OPUS_DECODER_RESET_START,
                       sizeof(OpusDecoder) - ((char *)&st->OPUS_DECODER_RESET_START - (char *)st));

            celt_decoder_ctl(celt_dec, (int32_t)OPUS_RESET_STATE);
            silk_InitDecoder();
            st->stream_channels = st->channels;
            st->frame_size = st->Fs / 400;
        } break;
        case OPUS_GET_SAMPLE_RATE_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            *value = st->Fs;
        } break;
        case OPUS_GET_PITCH_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            if (st->prev_mode == MODE_CELT_ONLY)
                ret = celt_decoder_ctl(celt_dec, (int32_t)value);
            else
                *value = s_silk_DecControlStruct->prevPitchLag;
        } break;
        case OPUS_GET_GAIN_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            *value = st->decode_gain;
        } break;
        case OPUS_SET_GAIN_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < -32768 || value > 32767) {
                goto bad_arg;
            }
            st->decode_gain = value;
        } break;
        case OPUS_GET_LAST_PACKET_DURATION_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            *value = st->last_packet_duration;
        } break;
        case OPUS_SET_PHASE_INVERSION_DISABLED_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 0 || value > 1) {
                goto bad_arg;
            }
            ret = celt_decoder_ctl(celt_dec, value);
        } break;
        case OPUS_GET_PHASE_INVERSION_DISABLED_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            ret = celt_decoder_ctl(celt_dec, (int32_t)value);
        } break;
        default:
            /*fprintf(stderr, "unknown opus_decoder_ctl() request: %d", request);*/
            ret = OPUS_UNIMPLEMENTED;
            break;
    }

    va_end(ap);
    return ret;
bad_arg:
    va_end(ap);
    return OPUS_BAD_ARG;
}
//----------------------------------------------------------------------------------------------------------------------

void opus_decoder_destroy(OpusDecoder *st) { free(st); }
//----------------------------------------------------------------------------------------------------------------------

int opus_packet_get_bandwidth(uint8_t *data) {
    int bandwidth;
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

int opus_packet_get_nb_channels(uint8_t *data) { return (data[0] & 0x4) ? 2 : 1; }
//----------------------------------------------------------------------------------------------------------------------

int opus_packet_get_nb_frames(uint8_t packet[], int32_t len) {
    int count;
    if (len < 1) return OPUS_BAD_ARG;
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

int opus_packet_get_nb_samples(uint8_t packet[], int32_t len, int32_t Fs) {
    int samples;
    int count = opus_packet_get_nb_frames(packet, len);

    if (count < 0) return count;

    samples = count * opus_packet_get_samples_per_frame(packet, Fs);
    /* Can't have more than 120 ms */
    if (samples * 25 > Fs * 3)
        return OPUS_INVALID_PACKET;
    else
        return samples;
}
//----------------------------------------------------------------------------------------------------------------------

int opus_decoder_get_nb_samples(const OpusDecoder *dec, uint8_t packet[], int32_t len) {
    return opus_packet_get_nb_samples(packet, len, dec->Fs);
}
//----------------------------------------------------------------------------------------------------------------------

int encode_size(int size, unsigned char *data) {
    if (size < 252) {
        data[0] = size;
        return 1;
    } else {
        data[0] = 252 + (size & 0x3);
        data[1] = (size - (int)data[0]) >> 2;
        return 2;
    }
}
//----------------------------------------------------------------------------------------------------------------------

static int parse_size(uint8_t *data, int32_t len, int16_t *size) {
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

int opus_packet_get_samples_per_frame(uint8_t *data, int32_t Fs) {
    int audiosize;
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

int opus_packet_parse_impl(uint8_t *data, int32_t len, int self_delimited, unsigned char *out_toc,
                           uint8_t *frames[48], int16_t size[48], int *payload_offset,
                           int32_t *packet_offset) {
    int i, bytes;
    int count;
    int cbr;
    unsigned char ch, toc;
    int framesize;
    int32_t last_size;
    int32_t pad = 0;
    uint8_t *data0 = data;

    if (size == NULL || len < 0) return OPUS_BAD_ARG;
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
            bytes = parse_size(data, len, size);
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
                int p;
                do {
                    int tmp;
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
                    bytes = parse_size(data, len, size + i);
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
        bytes = parse_size(data, len, size + count - 1);
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

    if (payload_offset) *payload_offset = (int)(data - data0);

    for (i = 0; i < count; i++) {
        if (frames) frames[i] = data;
        data += size[i];
    }

    if (packet_offset) *packet_offset = pad + (int32_t)(data - data0);

    if (out_toc) *out_toc = toc;

    return count;
}
//----------------------------------------------------------------------------------------------------------------------

int opus_packet_parse(uint8_t *data, int32_t len, unsigned char *out_toc, uint8_t *frames[48],
                      int16_t size[48], int *payload_offset) {
    return opus_packet_parse_impl(data, len, 0, out_toc, frames, size, payload_offset, NULL);
}
//----------------------------------------------------------------------------------------------------------------------
int validate_layout(OpusMSDecoder_t *layout) {
    int i, max_channel;

    max_channel = layout->nb_streams + layout->nb_coupled_streams;
    if (max_channel > 255) return 0;
    for (i = 0; i < layout->nb_channels; i++) {
        if (layout->mapping[i] >= max_channel && layout->mapping[i] != 255) return 0;
    }
    return 1;
}
//----------------------------------------------------------------------------------------------------------------------

int get_left_channel(OpusMSDecoder_t *layout, int stream_id, int prev) {
    int i;
    i = (prev < 0) ? 0 : prev + 1;
    for (; i < layout->nb_channels; i++) {
        if (layout->mapping[i] == stream_id * 2) return i;
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------

int get_right_channel(OpusMSDecoder_t *layout, int stream_id, int prev) {
    int i;
    i = (prev < 0) ? 0 : prev + 1;
    for (; i < layout->nb_channels; i++) {
        if (layout->mapping[i] == stream_id * 2 + 1) return i;
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------

int get_mono_channel(OpusMSDecoder_t *layout, int stream_id, int prev) {
    int i;
    i = (prev < 0) ? 0 : prev + 1;
    for (; i < layout->nb_channels; i++) {
        if (layout->mapping[i] == stream_id + layout->nb_coupled_streams) return i;
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------
int32_t opus_multistream_decoder_get_size(int nb_streams, int nb_coupled_streams) {
    log_i("opus_multistream_decoder_get_size %i, %i", nb_streams, nb_coupled_streams);
    int coupled_size;
    int mono_size;

    if (nb_streams < 1 || nb_coupled_streams > nb_streams || nb_coupled_streams < 0) return 0;
    coupled_size = opus_decoder_get_size(2);
    mono_size = opus_decoder_get_size(1);
    return align(sizeof(OpusMSDecoder_t)) + nb_coupled_streams * align(coupled_size) +
           (nb_streams - nb_coupled_streams) * align(mono_size);
}
//----------------------------------------------------------------------------------------------------------------------

int opus_multistream_decoder_init(OpusMSDecoder_t *st, int32_t Fs, int channels, int streams, int coupled_streams,
                                 const uint8_t *mapping) {
    int coupled_size;
    int mono_size;
    int i, ret;
    char *ptr;

    if ((channels > 255) || (channels < 1) || (coupled_streams > streams) || (streams < 1) || (coupled_streams < 0) ||
        (streams > 255 - coupled_streams))
        return OPUS_BAD_ARG;

    st->nb_channels = channels;
    st->nb_streams = streams;
    st->nb_coupled_streams = coupled_streams;

    for (i = 0; i < st->nb_channels; i++) st->mapping[i] = mapping[i];
    if (!validate_layout(st)) return OPUS_BAD_ARG;

    ptr = (char *)st + align(sizeof(OpusMSDecoder_t));
    coupled_size = opus_decoder_get_size(2);
    mono_size = opus_decoder_get_size(1);

    for (i = 0; i < st->nb_coupled_streams; i++) {
        ret = opus_decoder_init((OpusDecoder *)ptr, Fs, 2);
        if (ret != OPUS_OK) return ret;
        ptr += align(coupled_size);
    }
    for (; i < st->nb_streams; i++) {
        ret = opus_decoder_init((OpusDecoder *)ptr, Fs, 1);
        if (ret != OPUS_OK) return ret;
        ptr += align(mono_size);
    }
    return OPUS_OK;
}
//----------------------------------------------------------------------------------------------------------------------

OpusMSDecoder_t *opus_multistream_decoder_create(int32_t Fs, int channels, int streams, int coupled_streams,
                                               const uint8_t *mapping, int *error) {
    int ret;
    OpusMSDecoder_t *st;
    if ((channels > 255) || (channels < 1) || (coupled_streams > streams) || (streams < 1) || (coupled_streams < 0) ||
        (streams > 255 - coupled_streams)) {
        if (error) *error = OPUS_BAD_ARG;
        return NULL;
    }
    log_w("decoder get size %i", opus_multistream_decoder_get_size(streams, coupled_streams));
    st = (OpusMSDecoder_t *)ps_malloc(opus_multistream_decoder_get_size(streams, coupled_streams));
    if (st == NULL) {
        if (error) *error = OPUS_ALLOC_FAIL;
        return NULL;
    }
    ret = opus_multistream_decoder_init(st, Fs, channels, streams, coupled_streams, mapping);
    if (error) *error = ret;
    if (ret != OPUS_OK) {
        free(st);
        st = NULL;
    }
    return st;
}
//----------------------------------------------------------------------------------------------------------------------

static int opus_multistream_packet_validate(uint8_t *data, int32_t len, int nb_streams, int32_t Fs) {
    int s;
    int count;
    unsigned char toc;
    int16_t size[48];
    int samples = 0;
    int32_t packet_offset;

    for (s = 0; s < nb_streams; s++) {
        int tmp_samples;
        if (len <= 0) return OPUS_INVALID_PACKET;
        count = opus_packet_parse_impl(data, len, s != nb_streams - 1, &toc, NULL, size, NULL, &packet_offset);
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

int opus_multistream_decode_native(OpusMSDecoder_t *st, uint8_t *data, int32_t len, void *pcm,
                                   opus_copy_channel_out_func copy_channel_out, int frame_size) {
    //   log_i("opus_multistream_decode_native len %i", len);
    int32_t Fs;
    int coupled_size;
    int mono_size;
    int s, c;
    char *ptr;
    int do_plc = 0;
    VARDECL(int16_t, buf);
    if (frame_size <= 0) {
        return OPUS_BAD_ARG;
    }
    /* Limit frame_size to avoid excessive stack allocations. */
    opus_multistream_decoder_ctl(st, OPUS_GET_SAMPLE_RATE_REQUEST, &Fs);
    frame_size = min((int32_t)frame_size, Fs / 25 * 3);
    ALLOC(buf, 2 * frame_size, int16_t);
    ptr = (char *)st + align(sizeof(OpusMSDecoder_t));
    coupled_size = opus_decoder_get_size(2);
    mono_size = opus_decoder_get_size(1);

    if (len == 0) do_plc = 1;
    if (len < 0) {
        return OPUS_BAD_ARG;
    }
    if (!do_plc && len < 2 * st->nb_streams - 1) {
        return OPUS_INVALID_PACKET;
    }
    if (!do_plc) {
        int ret = opus_multistream_packet_validate(data, len, st->nb_streams, Fs);
        if (ret < 0) {
            return ret;
        } else if (ret > frame_size) {
            return OPUS_BUFFER_TOO_SMALL;
        }
    }
    for (s = 0; s < st->nb_streams; s++) {
        OpusDecoder *dec;
        int32_t packet_offset;
        int ret;

        dec = (OpusDecoder *)ptr;
        ptr += (s < st->nb_coupled_streams) ? align(coupled_size) : align(mono_size);

        if (!do_plc && len <= 0) {
            return OPUS_INTERNAL_ERROR;
        }
        packet_offset = 0;
        ret = opus_decode_native(dec, data, len, buf, frame_size, s != st->nb_streams - 1, &packet_offset);
        data += packet_offset;
        len -= packet_offset;
        if (ret <= 0) {
            return ret;
        }
        frame_size = ret;
        if (s < st->nb_coupled_streams) {
            int chan, prev;
            prev = -1;
            /* Copy "left" audio to the channel(s) where it belongs */
            while ((chan = get_left_channel(st, s, prev)) != -1) {
                (*copy_channel_out)(pcm, st->nb_channels, chan, buf, 2, frame_size, NULL);
                prev = chan;
            }
            prev = -1;
            /* Copy "right" audio to the channel(s) where it belongs */
            while ((chan = get_right_channel(st, s, prev)) != -1) {
                (*copy_channel_out)(pcm, st->nb_channels, chan, buf + 1, 2, frame_size, NULL);
                prev = chan;
            }
        } else {
            int chan, prev;
            prev = -1;
            /* Copy audio to the channel(s) where it belongs */
            while ((chan = get_mono_channel(st, s, prev)) != -1) {
                (*copy_channel_out)(pcm, st->nb_channels, chan, buf, 1, frame_size, NULL);
                prev = chan;
            }
        }
    }
    /* Handle muted channels */
    for (c = 0; c < st->nb_channels; c++) {
        if (st->mapping[c] == 255) {
            (*copy_channel_out)(pcm, st->nb_channels, c, NULL, 0, frame_size, NULL);
        }
    }

    return frame_size;
}
//----------------------------------------------------------------------------------------------------------------------

static void opus_copy_channel_out_short(void *dst, int dst_stride, int dst_channel, const int16_t *src, int src_stride,
                                        int frame_size, void *user_data) {
    int16_t *short_dst;
    int32_t i;
    (void)user_data;
    short_dst = (int16_t *)dst;
    if (src != NULL) {
        for (i = 0; i < frame_size; i++) short_dst[i * dst_stride + dst_channel] = src[i * src_stride];

    } else {
        for (i = 0; i < frame_size; i++) short_dst[i * dst_stride + dst_channel] = 0;
    }
}
//----------------------------------------------------------------------------------------------------------------------

int opus_multistream_decode(OpusMSDecoder_t *st, uint8_t *data, int32_t len, int16_t *pcm, int frame_size) {
    return opus_multistream_decode_native(st, data, len, pcm, opus_copy_channel_out_short, frame_size);
}
//----------------------------------------------------------------------------------------------------------------------

int opus_multistream_decoder_ctl_va_list(OpusMSDecoder_t *st, int request, va_list ap) {
    int coupled_size, mono_size;
    char *ptr;
    int ret = OPUS_OK;

    coupled_size = opus_decoder_get_size(2);
    mono_size = opus_decoder_get_size(1);
    ptr = (char *)st + align(sizeof(OpusMSDecoder_t));
    switch (request) {
        case OPUS_GET_BANDWIDTH_REQUEST:
        case OPUS_GET_SAMPLE_RATE_REQUEST:
        case OPUS_GET_GAIN_REQUEST:
        case OPUS_GET_LAST_PACKET_DURATION_REQUEST:
        case OPUS_GET_PHASE_INVERSION_DISABLED_REQUEST: {
            OpusDecoder *dec;
            /* For int32* GET params, just query the first stream */
            int32_t *value = va_arg(ap, int32_t *);
            dec = (OpusDecoder *)ptr;
            ret = opus_decoder_ctl(dec, request, value);
        } break;
        case OPUS_GET_FINAL_RANGE_REQUEST: {
            int s;
            uint32_t *value = va_arg(ap, uint32_t *);
            uint32_t tmp;
            if (!value) {
                goto bad_arg;
            }
            *value = 0;
            for (s = 0; s < st->nb_streams; s++) {
                OpusDecoder *dec;
                dec = (OpusDecoder *)ptr;
                if (s < st->nb_coupled_streams)
                    ptr += align(coupled_size);
                else
                    ptr += align(mono_size);
                ret = opus_decoder_ctl(dec, request, &tmp);
                if (ret != OPUS_OK) break;
                *value ^= tmp;
            }
        } break;
        case OPUS_RESET_STATE: {
            int s;
            for (s = 0; s < st->nb_streams; s++) {
                OpusDecoder *dec;

                dec = (OpusDecoder *)ptr;
                if (s < st->nb_coupled_streams)
                    ptr += align(coupled_size);
                else
                    ptr += align(mono_size);
                ret = opus_decoder_ctl(dec, OPUS_RESET_STATE);
                if (ret != OPUS_OK) break;
            }
        } break;
        case OPUS_MULTISTREAM_GET_DECODER_STATE_REQUEST: {
            int s;
            int32_t stream_id;
            OpusDecoder **value;
            stream_id = va_arg(ap, int32_t);
            if (stream_id < 0 || stream_id >= st->nb_streams) goto bad_arg;
            value = va_arg(ap, OpusDecoder **);
            if (!value) {
                goto bad_arg;
            }
            for (s = 0; s < stream_id; s++) {
                if (s < st->nb_coupled_streams)
                    ptr += align(coupled_size);
                else
                    ptr += align(mono_size);
            }
            *value = (OpusDecoder *)ptr;
        } break;
        case OPUS_SET_GAIN_REQUEST:
        case OPUS_SET_PHASE_INVERSION_DISABLED_REQUEST: {
            int s;
            /* This works for int32 params */
            int32_t value = va_arg(ap, int32_t);
            for (s = 0; s < st->nb_streams; s++) {
                OpusDecoder *dec;

                dec = (OpusDecoder *)ptr;
                if (s < st->nb_coupled_streams)
                    ptr += align(coupled_size);
                else
                    ptr += align(mono_size);
                ret = opus_decoder_ctl(dec, request, value);
                if (ret != OPUS_OK) break;
            }
        } break;
        default:
            ret = OPUS_UNIMPLEMENTED;
            break;
    }
    return ret;
bad_arg:
    return OPUS_BAD_ARG;
}
//----------------------------------------------------------------------------------------------------------------------

int opus_multistream_decoder_ctl(OpusMSDecoder_t *st, int request, ...) {
    int ret;
    va_list ap;
    va_start(ap, request);
    ret = opus_multistream_decoder_ctl_va_list(st, request, ap);
    va_end(ap);
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

void opus_multistream_decoder_destroy(OpusMSDecoder_t *st) { free(st); }
//----------------------------------------------------------------------------------------------------------------------
