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

#pragma once

#include <stdint.h>

#include "../celt/celt.h"
#include "silk.h"
#include "errors.h"
#include "typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SILK_MAX_FRAMES_PER_PACKET 3
/* Decoder API flags */
#define FLAG_DECODE_NORMAL                      0
#define FLAG_PACKET_LOST                        1
#define FLAG_DECODE_LBRR                        2

/* Struct for TOC (Table of Contents) */
typedef struct {
    int32_t VADFlag;                              /* Voice activity for packet                            */
    int32_t VADFlags[SILK_MAX_FRAMES_PER_PACKET]; /* Voice activity for each frame in packet              */
    int32_t inbandFECFlag;                        /* Flag indicating if packet contains in-band FEC       */
} silk_TOC_struct;

typedef struct {         /* Structure for controlling encoder operation */
    int32_t nChannelsAPI;/* Number of channels; 1/2 */
    int32_t nChannelsInternal;/* Number of channels; 1/2 */
    int32_t API_sampleRate; /* Input signal sampling rate in Hertz; 8000/12000/16000/24000/32000/44100/48000 */
    int32_t maxInternalSampleRate; /* Maximum internal sampling rate in Hertz; 8000/12000/16000 */
    int32_t minInternalSampleRate; /* Minimum internal sampling rate in Hertz; 8000/12000/16000 */
    int32_t desiredInternalSampleRate; /* Soft request for internal sampling rate in Hertz; 8000/12000/16000 */
    int32_t payloadSize_ms;/* Number of samples per packet in milliseconds; 10/20/40/60 */
    int32_t bitRate;/*  Bitrate during active speech in bits/second; internally limited */
    int32_t packetLossPercentage;/* Uplink packet loss in percent (0-100) */
    int32_t complexity; /* Complexity mode; 0 is lowest, 10 is highest complexity */
    int32_t useInBandFEC; /* Flag to enable in-band Forward Error Correction (FEC); 0/1 */
    int32_t LBRR_coded; /* Flag to actually code in-band Forward Error Correction (FEC) in the current packet; 0/1 */
    int32_t useDTX; /* Flag to enable discontinuous transmission (DTX); 0/1 */
    int32_t useCBR; /* Flag to use constant bitrate */
    int32_t maxBits; /* Maximum number of bits allowed for the frame */
    int32_t toMono; /* Causes a smooth downmix to mono */
    int32_t opusCanSwitch; /* Opus encoder is allowing us to switch bandwidth  */
    int32_t reducedDependency; /* Make frames as independent as possible (but still use LPC)*/
    int32_t internalSampleRate; /* Internal sampling rate used, in Hertz; 8000/12000/16000 */
    int32_t allowBandwidthSwitch; /* Flag that bandwidth switching is allowed */
    int32_t inWBmodeWithoutVariableLP; /* use for switching between WB/SWB/FB */
    int32_t stereoWidth_Q14; /* Stereo width */
    int32_t switchReady; /* Tells the Opus encoder we're ready to switch */
    int32_t signalType; /* SILK Signal type */
    int32_t offset; /* SILK offset (dithering) */
} silk_EncControlStruct;



#if 0
/**************************************/
/* Get table of contents for a packet */
/**************************************/
int32_t silk_get_TOC(
    const uint8_t      *payload,           /* I    Payload data                                */
    const int32_t      nBytesIn,           /* I    Number of input bytes                       */
    const int32_t      nFramesPerPayload,  /* I    Number of SILK frames per payload           */
    silk_TOC_struct    *Silk_TOC           /* O    Type of content                             */
);
#endif



/**************************************************************************/
/* Structure for controlling decoder operation and reading decoder status */
/**************************************************************************/
typedef struct {
    /* I:   Number of channels; 1/2                                                         */
    int32_t nChannelsAPI;

    /* I:   Number of channels; 1/2                                                         */
    int32_t nChannelsInternal;

    /* I:   Output signal sampling rate in Hertz; 8000/12000/16000/24000/32000/44100/48000  */
    int32_t API_sampleRate;

    /* I:   Internal sampling rate used, in Hertz; 8000/12000/16000                         */
    int32_t internalSampleRate;

    /* I:   Number of samples per packet in milliseconds; 10/20/40/60                       */
    int32_t payloadSize_ms;

    /* O:   Pitch lag of previous frame (0 if unvoiced), measured in samples at 48 kHz      */
    int32_t prevPitchLag;
} silk_DecControlStruct;


// prototypes and inlines
int32_t silk_Get_Encoder_Size(int32_t *encSizeBytes);

int32_t silk_InitEncoder(void *encState, int arch, silk_EncControlStruct *encStatus);

int32_t silk_Encode(void *encState, silk_EncControlStruct  *encControl, const int16_t *samplesIn, int32_t  nSamplesIn,
                   ec_enc *psRangeEnc, int32_t *nBytesOut, const int32_t prefillFlag, int  activity);
int32_t silk_Get_Decoder_Size(int32_t *decSizeBytes);
int32_t silk_InitDecoder(void *decState);
int32_t silk_Decode(void* decState, silk_DecControlStruct* decControl, int32_t lostFlag, int32_t newPacketFlag,
                    ec_dec *psRangeDec, int16_t *samplesOut, int32_t *nSamplesOut, int arch);




#ifdef __cplusplus
}
#endif


