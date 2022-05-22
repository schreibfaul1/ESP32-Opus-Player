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
#include "../libogg/ogg.h"
#include "../libopus/opus_multistream.h"


typedef int16_t op_sample;

#define OP_ASSERT(_cond)

#define OP_CLAMP(_lo,_x,_hi) (_max(_lo,_min(_x,_hi)))

typedef struct OpusMemStream OpusMemStream;

#define OP_MEM_SIZE_MAX (~(size_t)0>>1)
#define OP_MEM_DIFF_MAX ((ptrdiff_t)OP_MEM_SIZE_MAX)



/*Advance a file offset by the given amount, clamping against INT64_MAX.
  This is used to advance a known offset by things like OP_CHUNK_SIZE or
   OP_PAGE_SIZE_MAX, while making sure to avoid signed overflow.
  It assumes that both _offset and _amount are non-negative.*/
#define OP_ADV_OFFSET(_offset,_amount) \
 (_min(_offset,INT64_MAX-(_amount))+(_amount))

#define OP_NCHANNELS_MAX (2)
#define OPUS_CHANNEL_COUNT_MAX (255)

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

typedef int (*op_read_func)(void *_stream,unsigned char *_ptr,int _nbytes);
typedef int (*op_seek_func)(void *_stream,int64_t _offset,int _whence);
typedef int64_t (*op_tell_func)(void *_stream);
typedef int (*op_close_func)(void *_stream);

typedef int (*op_decode_cb_func)(void *_ctx,OpusMSDecoder *_decoder,void *_pcm, const ogg_packet *_op,
             int _nsamples,int _nchannels,int _format,int _li);

typedef struct OpusHead{
  int           version;
  int           channel_count;
  unsigned      pre_skip;
  uint32_t      input_sample_rate;
  int           output_gain;
  int           mapping_family;
  int           stream_count;
  int           coupled_count;
  unsigned char mapping[OPUS_CHANNEL_COUNT_MAX];
} OpusHead_t;

typedef struct OpusTags{
  char        **user_comments;
  int          *comment_lengths;
  int           comments;
  char         *vendor;
} OpusTags_t;

typedef struct OpusPictureTag{
  int32_t       type;
  char         *mime_type;
  char         *description;
  uint32_t      width;
  uint32_t      height;
  uint32_t      depth;
  uint32_t      colors;
  uint32_t      data_length;
  unsigned char *data;
  int           format;
} OpusPictureTag_t;

typedef struct OpusServerInfo{
  char        *name;
  char        *description;
  char        *genre;
  char        *url;
  char        *server;
  char        *content_type;
  int32_t      bitrate_kbps;
  int          is_public;
  int          is_ssl;
}OpusServerInfo_t;

typedef struct OpusFileCallbacks{
  op_read_func  read;
  op_seek_func  seek;
  op_tell_func  tell;
  op_close_func close;
} OpusFileCallbacks_t;

typedef struct OggOpusLink{
  int64_t           offset;
  int64_t           data_offset;
  int64_t           end_offset;
  int64_t       pcm_file_offset;
  int64_t       pcm_end;
  int64_t       pcm_start;
  uint32_t      serialno;
  OpusHead_t        head;
  OpusTags_t        tags;
} OggOpusLink_t;

typedef struct OggOpusFile{
  OpusFileCallbacks_t  callbacks;
  void             *stream;
  int               seekable;
  int               nlinks;
  OggOpusLink_t    *links;
  int               nserialnos;
  int               cserialnos;
  uint32_t         *serialnos;
  int64_t           offset;
  int64_t           end;
  ogg_sync_state    oy;
  int               ready_state;
  int               cur_link;
  int32_t           cur_discard_count;
  int64_t           prev_packet_gp;
  int64_t           prev_page_offset;
  int64_t           bytes_tracked;
  int64_t           samples_tracked;
  ogg_stream_state  os;
  ogg_packet        op[255];
  int               op_pos;
  int               op_count;
  OpusMSDecoder    *od;
  op_decode_cb_func decode_cb;
  void             *decode_cb_ctx;
  int               od_stream_count;
  int               od_coupled_count;
  int               od_channel_count;
  unsigned char     od_mapping[OP_NCHANNELS_MAX];
  op_sample        *od_buffer;
  int               od_buffer_pos;
  int               od_buffer_size;
  int               gain_type;
  int32_t           gain_offset_q8;
} OggOpusFile_t;

struct OpusMemStream {
    const unsigned char *data;
    ptrdiff_t size;
    ptrdiff_t pos;
};


int op_strncasecmp(const char *_a,const char *_b,int _n);


/**@cond PRIVATE*/

/*Enable special features for gcc and gcc-compatible compilers.*/
//
//#  if defined(__GNUC__)&&defined(__GNUC_MINOR__)
//#   define OP_GNUC_PREREQ(_maj,_min) \
// ((__GNUC__<<16)+__GNUC_MINOR__>=((_maj)<<16)+(_min))
//
//
//# if OP_GNUC_PREREQ(4,0)
//#  pragma GCC visibility push(default)
//# endif

typedef struct OggOpusFile       OggOpusFile;


#define OP_FALSE         (-1)
#define OP_EOF           (-2)
#define OP_HOLE          (-3)
#define OP_EREAD         (-128)
#define OP_EFAULT        (-129)
#define OP_EIMPL         (-130)

#define OP_EINVAL        (-131)
#define OP_ENOTFORMAT    (-132)
#define OP_EBADHEADER    (-133)
/**The ID header contained an unrecognized version number.*/
#define OP_EVERSION      (-134)
/*Currently not used at all.*/
#define OP_ENOTAUDIO     (-135)
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




int opus_head_parse(OpusHead_t *_head, const unsigned char *_data,size_t _len);
int64_t opus_granule_sample(const OpusHead_t *_head,int64_t _gp);
int opus_tags_parse(OpusTags_t *_tags, const unsigned char *_data,size_t _len);
int opus_tags_copy(OpusTags_t *_dst,const OpusTags_t *_src);
void opus_tags_init(OpusTags_t *_tags);
int opus_tags_add(OpusTags_t *_tags,const char *_tag,const char *_value);
int opus_tags_add_comment(OpusTags_t *_tags,const char *_comment);
int opus_tags_set_binary_suffix(OpusTags_t *_tags, const unsigned char *_data,int _len);
const char *opus_tags_query(const OpusTags_t *_tags,const char *_tag,int _count);
int opus_tags_query_count(const OpusTags_t *_tags,const char *_tag);
const unsigned char *opus_tags_get_binary_suffix(const OpusTags_t *_tags, int *_len);
int opus_tags_get_album_gain(const OpusTags_t *_tags,int *_gain_q8);
int opus_tags_get_track_gain(const OpusTags_t *_tags,int *_gain_q8);
void opus_tags_clear(OpusTags_t *_tags);
int opus_tagcompare(const char *_tag_name,const char *_comment);
int opus_tagncompare(const char *_tag_name,int _tag_len,const char *_comment);
int opus_picture_tag_parse(OpusPictureTag_t *_pic, const char *_tag);
void opus_picture_tag_init(OpusPictureTag_t *_pic);
void opus_picture_tag_clear(OpusPictureTag_t *_pic);
void *op_fopen(OpusFileCallbacks_t *_cb, const char *_path,const char *_mode);
void *op_fdopen(OpusFileCallbacks_t *_cb, int _fd,const char *_mode);
void *op_freopen(OpusFileCallbacks_t *_cb, const char *_path,const char *_mode,void *_stream);
void *op_mem_stream_create(OpusFileCallbacks_t *_cb, const unsigned char *_data,size_t _size);
void *op_url_stream_vcreate(OpusFileCallbacks_t *_cb, const char *_url,va_list _ap);
void *op_url_stream_create(OpusFileCallbacks_t *_cb, const char *_url,...);
OggOpusFile *op_open_file(const char *_path,int *_error);
OggOpusFile *op_open_memory(const unsigned char *_data, size_t _size,int *_error);
OggOpusFile *op_vopen_url(const char *_url, int *_error,va_list _ap);
OggOpusFile *op_open_url(const char *_url, int *_error,...);
OggOpusFile *op_open_callbacks(void *_stream,
const OpusFileCallbacks_t *_cb,const unsigned char *_initial_data, size_t _initial_bytes,int *_error);

OggOpusFile *op_test_file(const char *_path,int *_error);
int op_test(OpusHead_t *_head, const unsigned char *_initial_data,size_t _initial_bytes);
OggOpusFile *op_test_memory(const unsigned char *_data, size_t _size,int *_error);
OggOpusFile *op_vtest_url(const char *_url, int *_error,va_list _ap);
OggOpusFile *op_test_url(const char *_url, int *_error,...);
OggOpusFile *op_test_callbacks(void *_stream, const OpusFileCallbacks_t *_cb,const unsigned char *_initial_data,
                               size_t _initial_bytes,int *_error) ;
int op_test_open(OggOpusFile *_of);
void op_free(OggOpusFile *_of);
int op_seekable(const OggOpusFile *_of);
int op_link_count(const OggOpusFile *_of);
uint32_t op_serialno(const OggOpusFile *_of,int _li);
int op_channel_count(const OggOpusFile *_of,int _li);
int64_t op_raw_total(const OggOpusFile *_of,int _li);
int64_t op_pcm_total(const OggOpusFile *_of,int _li);
const OpusHead_t *op_head(const OggOpusFile *_of,int _li);
const OpusTags_t *op_tags(const OggOpusFile *_of,int _li);
int op_current_link(const OggOpusFile *_of);
int32_t op_bitrate(const OggOpusFile *_of,int _li);
int32_t op_bitrate_instant(OggOpusFile *_of);
int64_t op_raw_tell(const OggOpusFile *_of);
int64_t op_pcm_tell(const OggOpusFile *_of);
int op_raw_seek(OggOpusFile *_of,int64_t _byte_offset);
int op_pcm_seek(OggOpusFile *_of,int64_t _pcm_offset);

#define OP_DEC_FORMAT_SHORT (7008)
#define OP_DEC_FORMAT_FLOAT (7040)
#define OP_DEC_USE_DEFAULT  (6720)



void op_set_decode_callback(OggOpusFile *_of, op_decode_cb_func _decode_cb,void *_ctx);

#define OP_HEADER_GAIN   (0)
#define OP_ALBUM_GAIN    (3007)
#define OP_TRACK_GAIN    (3008)
#define OP_ABSOLUTE_GAIN (3009)

int op_set_gain_offset(OggOpusFile *_of, int _gain_type,int32_t _gain_offset_q8);
void op_set_dither_enabled(OggOpusFile *_of,int _enabled);
int op_read(OggOpusFile *_of, int16_t *_pcm,int _buf_size,int *_li);
int op_read_float(OggOpusFile *_of, float *_pcm,int _buf_size,int *_li);
int op_read_stereo(OggOpusFile *_of, int16_t *_pcm,int _buf_size);
int op_read_float_stereo(OggOpusFile *_of, float *_pcm,int _buf_size);


