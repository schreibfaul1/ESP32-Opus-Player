/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE libopusfile SOFTWARE CODEC SOURCE CODE. *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE libopusfile SOURCE CODE IS (C) COPYRIGHT 1994-2012           *
 * by the Xiph.Org Foundation and contributors https://xiph.org/    *
 *                                                                  *
 ********************************************************************/

#pragma once

#include "Arduino.h"
#include "ogg.h"
#include "opus_decoder.h"

extern __attribute__((weak)) int32_t SD_read(uint8_t* buff, int32_t nbytes);

/*Advance a file offset by the given amount, clamping against INT64_MAX.
  It assumes that both _offset and _amount are non-negative.*/
#define OP_ADV_OFFSET(_offset,_amount) \
 (_min(_offset,INT64_MAX-(_amount))+(_amount))

#define OP_NCHANNELS_MAX (2)
#define OPUS_CHANNEL_COUNT_MAX (2)

/*Initial state.*/
# define  OP_NOTOPEN   (0)
/*We've found the first Opus stream in the first link.*/
# define  OP_PARTOPEN  (1)
# define  OP_OPENED    (2)
/*We've found the first Opus stream in the current link.*/
# define  OP_STREAMSET (3)
/*We've initialized the decoder for the chosen Opus stream in the current
   link.*/
# define  OP_INITSET   (4)

typedef int32_t (*op_read_func)(uint8_t *_ptr,int32_t _nbytes);

typedef struct OpusHead{
    int32_t           version;
    int32_t           channel_count;
    uint32_t          pre_skip;
    uint32_t          input_sample_rate;
    int32_t           output_gain;
    int32_t           mapping_family;
    int32_t           stream_count;
    int32_t           coupled_count;
    uint8_t           mapping[OPUS_CHANNEL_COUNT_MAX];
} OpusHead_t;

typedef struct OpusTags{
    char            **user_comments;
    int32_t          *comment_lengths;
    int32_t           comments;
    char             *vendor;
} OpusTags_t;

typedef struct OggOpusLink{
    int64_t           offset;
    int64_t           data_offset;
    int64_t           end_offset;
    int64_t           pcm_file_offset;
    int64_t           pcm_end;
    int64_t           pcm_start;
    uint32_t          serialno;
    OpusHead_t        head;
    OpusTags_t        tags;
} OggOpusLink_t;

typedef struct OggOpusFile{
    int32_t           seekable;
    int32_t           nlinks;
    int32_t           nserialnos;
    int32_t           cserialnos;
    uint32_t         *serialnos;
    int64_t           offset;
    int64_t           end;
    ogg_sync_state    oy;
    int32_t           ready_state;
    int32_t           cur_link;
    int32_t           cur_discard_count;
    int64_t           prev_packet_gp;
    int64_t           prev_page_offset;
    int64_t           bytes_tracked;
    int64_t           samples_tracked;
    ogg_stream_state  os;
    ogg_packet        op[255];
    int32_t           op_pos;
    int32_t           op_count;
    void             *decode_cb_ctx;
    int32_t           od_stream_count;
    int32_t           od_coupled_count;
    int32_t           od_channel_count;
    uint8_t           od_mapping[OP_NCHANNELS_MAX];
    int16_t          *od_buffer;
    int32_t           od_buffer_pos;
    int32_t           od_buffer_size;
    int32_t           gain_type;
    int32_t           gain_offset_q8;
} OggOpusFile_t;

#define OP_FALSE         (-1)
#define OP_EOF           (-2)
#define OP_HOLE          (-3)
#define OP_EREAD         (-128)
#define OP_EFAULT        (-129)
#define OP_EIMPL         (-130)

#define OP_EINVAL        (-131)
#define OP_ENOTFORMAT    (-132)
#define OP_EBADHEADER    (-133)
#define OP_EVERSION      (-134) /**The ID header contained an unrecognized version number.*/
#define OP_ENOTAUDIO     (-135) /*Currently not used at all.*/
/**An audio packet failed to decode properly.
   This is usually caused by a multistream Ogg packet where the durations of
    the individual Opus packets contained in it are not all the same.*/
#define OP_EBADPACKET    (-136)
/**We failed to find data we had seen before, or the bitstream structure was
    sufficiently malformed that seeking to the target destination was
    impossible.*/
#define OP_EBADLINK      (-137)
/**An operation that requires seeking was requested on an unseekable stream.*/
#define OP_ENOSEEK       (-138)
/**The first or last granule position of a link failed basic validity checks.*/
#define OP_EBADTIMESTAMP (-139)

#define OP_PIC_FORMAT_UNKNOWN (-1)
#define OP_PIC_FORMAT_URL     (0)
#define OP_PIC_FORMAT_JPEG    (1)
#define OP_PIC_FORMAT_PNG     (2)
#define OP_PIC_FORMAT_GIF     (3)

int32_t opus_head_parse(OpusHead_t *_head, const uint8_t *_data,size_t _len);

//OggOpusFile_t *op_open_file(const char *_path,int32_t *_error);
OggOpusFile_t *opus_init_decoder();

#define OP_DEC_FORMAT_SHORT (7008)
#define OP_DEC_FORMAT_FLOAT (7040)
#define OP_DEC_USE_DEFAULT  (6720)


#define OP_HEADER_GAIN   (0)
#define OP_ALBUM_GAIN    (3007)
#define OP_TRACK_GAIN    (3008)
#define OP_ABSOLUTE_GAIN (3009)

int32_t op_read_stereo(int16_t *_pcm,int32_t _buf_size);


