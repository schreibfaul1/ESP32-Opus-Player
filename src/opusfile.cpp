/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE libopusfile SOFTWARE CODEC SOURCE CODE. *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE libopusfile SOURCE CODE IS (C) COPYRIGHT 1994-2020           *
 * by the Xiph.Org Foundation and contributors https://xiph.org/    *
 *                                                                  *
 ********************************************************************/

// #include "config.h"
// #include "internal.h"
#include "Arduino.h"
#include "opusfile.h"



OggOpusFile_t    *m_OggOpusFile;
OggOpusLink_t    *m_OggOpusLink;
OpusMSDecoder_t  *m_od;

#define OP_CHUNK_SIZE     (1024 * 8)
#define OP_READ_SIZE      (2048)

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
/* Many, many internal helpers. The intention is not to be confusing. Rampant duplication and monolithic function
   implementation (though we do have some large, omnibus functions still) would be harder to understand anyway.
   The high level functions are last. Begin grokking near the end of the file if you prefer to read things top-down.*/

/* The read/seek functions track absolute position within the stream.*/
/* Read a little more data from the file/pipe into the ogg_sync framer. _nbytes: The maximum number of bytes to read.
   Return: A positive number of bytes read on success, 0 on end-of-file, or a negative value on failure.*/
int op_get_data(int _nbytes) {
    unsigned char *buffer;
    int nbytes;
    assert(_nbytes>0);
    buffer = (unsigned char*) ogg_sync_buffer(&m_OggOpusFile->oy, _nbytes);
    if(SD_read) nbytes = SD_read(buffer, _nbytes);
//    log_i("nbytes gelesen %i", nbytes);
    assert(nbytes<=_nbytes);
    if(nbytes > 0) ogg_sync_wrote(&m_OggOpusFile->oy, nbytes);
    return nbytes;
}
//----------------------------------------------------------------------------------------------------------------------
/* Get the current position indicator of the underlying stream. This should be the same as the value reported
   by tell().*/
int64_t op_position() {
    /*The current position indicator is _not_ simply offset.
     We may also have unprocessed, buffered data in the sync state.*/
    return m_OggOpusFile->offset + m_OggOpusFile->oy.fill - m_OggOpusFile->oy.returned;
}
//----------------------------------------------------------------------------------------------------------------------
/*From the head of the stream, get the next page. _boundary specifies if the function is allowed to fetch more data
  from the stream (and how much) or only use internally buffered data.
  _boundary: -1: Unbounded search. 0: Read no additional data. Use only cached data.
  n: Search for the start of a new page up to file position n. Return: n>=0: Found a page at absolute offset n.
  OP_FALSE:   Hit the _boundary limit.
  OP_EREAD:   An underlying read operation failed.
  OP_BADLINK: We hit end-of-file before reaching _boundary.*/
int64_t op_get_next_page( ogg_page *_og, int64_t _boundary) {
    while(_boundary <= 0 || m_OggOpusFile->offset < _boundary) {
        int more;
        more = ogg_sync_pageseek(&m_OggOpusFile->oy, _og);
        /*Skipped (-more) bytes.*/
        if(more < 0)
            m_OggOpusFile->offset -= more;
        else if(more == 0) {
            int read_nbytes;
            int ret;
            /*Send more paramedics.*/
            if(!_boundary) return OP_FALSE;
            if(_boundary < 0)
                read_nbytes = OP_READ_SIZE;
            else {
                int64_t position;
                position = op_position();
                if(position >= _boundary) return OP_FALSE;
                read_nbytes = (int) _min(_boundary - position, OP_READ_SIZE);
            }
            ret = op_get_data(read_nbytes);
            if(ret < 0) return OP_EREAD;
            if(ret == 0) {
                /*Only fail cleanly on EOF if we didn't have a known boundary.
                 Otherwise, we should have been able to reach that boundary, and this
                 is a fatal error.*/
                return (_boundary < 0) ? OP_FALSE : OP_EBADLINK;
            }
        }
        else {
            /*Got a page.
             Return the page start offset and advance the internal offset past the
             page end.*/
            int64_t page_offset;
            page_offset = m_OggOpusFile->offset;
            m_OggOpusFile->offset += more;
            assert(page_offset>=0);
            return page_offset;
        }
    }
    return OP_FALSE;
}
//----------------------------------------------------------------------------------------------------------------------
int op_add_serialno(const ogg_page *_og, uint32_t **_serialnos, int *_nserialnos, int *_cserialnos) {
    uint32_t *serialnos;
    int nserialnos;
    int cserialnos;
    uint32_t s;
    s = getSerialNo(_og);
    serialnos = *_serialnos;
    nserialnos = *_nserialnos;
    cserialnos = *_cserialnos;
    if(nserialnos >= cserialnos) {
        if((cserialnos > INT_MAX / (int) sizeof(*serialnos) - 1) >> 1) {
            return OP_EFAULT;
        }
        cserialnos = 2 * cserialnos + 1;
        assert(nserialnos<cserialnos);
        serialnos = (uint32_t*) realloc(serialnos, sizeof(*serialnos) * cserialnos);
        if(serialnos==NULL) return OP_EFAULT;
    }
    serialnos[nserialnos++] = s;
    *_serialnos = serialnos;
    *_nserialnos = nserialnos;
    *_cserialnos = cserialnos;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int op_lookup_page_serialno(const ogg_page *_og, const uint32_t *_serialnos, int _nserialnos) {

    int serialNo = getSerialNo(_og);
    int i;
    for(i = 0; i < _nserialnos && _serialnos[i] != serialNo; i++)
        ;
    return i < _nserialnos;
}
//----------------------------------------------------------------------------------------------------------------------
/* We use this to remember the pages we found while enumerating the links of a chained stream.
   We keep track of the starting and ending offsets, as well as the point we started searching from, so we know where
   to bisect. We also keep the serial number, so we can tell if the page belonged to the current link or not, as well
   as the granule position, to aid in estimating the start of the link.*/
typedef struct OpusSeekRecord {
    int64_t search_start;   /*The earliest byte we know of such that reading forward from it causes capture to be
                             regained at this page.*/
    int64_t offset;         /*The offset of this page.*/
    int32_t size;           /*The size of this page.*/
    uint32_t serialno;  /*The serial number of this page.*/
    int64_t gp;         /*The granule position of this page.*/
}OpusSeekRecord_t;
//----------------------------------------------------------------------------------------------------------------------
/*Uses the local ogg_stream storage in m_OggOpusFile.  This is important for non-streaming input sources.*/
int op_fetch_headers_impl(OpusHead_t *_head, OpusTags_t *_tags, uint32_t **_serialnos,
                                 int *_nserialnos, int *_cserialnos, ogg_page *_og) {
    ogg_packet op;
    int ret;
    if(_serialnos != NULL) *_nserialnos = 0;
    /*Extract the serialnos of all BOS pages plus the first set of Opus headers
     we see in the link.*/
    while(ogg_page_bos(_og)) {
        if(_serialnos != NULL) {
            if(op_lookup_page_serialno(_og, *_serialnos, *_nserialnos)) {
                /*A dupe serialnumber in an initial header packet set==invalid stream.*/
                return OP_EBADHEADER;
            }
            ret = op_add_serialno(_og, _serialnos, _nserialnos, _cserialnos);
            if(ret < 0) return ret;
        }
        if(m_OggOpusFile->ready_state < OP_STREAMSET) {
            /*We don't have an Opus stream in this link yet, so begin prospective
             stream setup.
             We need a stream to get packets.*/
            ogg_stream_reset_serialno(&m_OggOpusFile->os, getSerialNo(_og));
            ogg_stream_pagein(&m_OggOpusFile->os, _og);
            if(ogg_stream_packetout(&m_OggOpusFile->os, &op) > 0) {
                ret = opus_head_parse(_head, op.packet, op.bytes);
                /*Found a valid Opus header.
                 Continue setup.*/
                if(ret >= 0)
                    m_OggOpusFile->ready_state = OP_STREAMSET;
                /*If it's just a stream type we don't recognize, ignore it.
                 Everything else is fatal.*/
                else if(ret != OP_ENOTFORMAT) return ret;
            }
            /*TODO: Should a BOS page with no packets be an error?*/
        }
        /*Get the next page.
         No need to clamp the boundary offset against m_OggOpusFile->end, as all errors
         become OP_ENOTFORMAT or OP_EBADHEADER.*/
        if(op_get_next_page(_og, OP_ADV_OFFSET(m_OggOpusFile->offset,OP_CHUNK_SIZE))<0) {
            return m_OggOpusFile->ready_state < OP_STREAMSET ? OP_ENOTFORMAT : OP_EBADHEADER;
        }
    }
    if(m_OggOpusFile->ready_state!=OP_STREAMSET) return OP_ENOTFORMAT;
    /*If the first non-header page belonged to our Opus stream, submit it.*/
    if(m_OggOpusFile->os.serialno == getSerialNo(_og)) ogg_stream_pagein(&m_OggOpusFile->os, _og);
    /*Loop getting packets.*/
    for(;;) {
        switch(ogg_stream_packetout(&m_OggOpusFile->os, &op)){
            case 0: {
                /*Loop getting pages.*/
                for(;;) {
                    /*No need to clamp the boundary offset against m_OggOpusFile->end, as all
                     errors become OP_EBADHEADER.*/
                    if(op_get_next_page(_og, OP_ADV_OFFSET(m_OggOpusFile->offset,OP_CHUNK_SIZE))<0) {
                        return OP_EBADHEADER;
                    }
                    /*If this page belongs to the correct stream, go parse it.*/
                    if(m_OggOpusFile->os.serialno == getSerialNo(_og)) {
                        ogg_stream_pagein(&m_OggOpusFile->os, _og);
                        break;
                    }
                    /*If the link ends before we see the Opus comment header, abort.*/
                    if(ogg_page_bos(_og)) return OP_EBADHEADER;
                    /*Otherwise, keep looking.*/
                }
            }
                break;
                /*We shouldn't get a hole in the headers!*/
            case -1:
                return OP_EBADHEADER;
            default: {
                /*Got a packet.
                 It should be the comment header.*/
                ret = 0;// opus_tags_parse(_tags, op.packet, op.bytes);
                if(ret < 0) return ret;
                /*Make sure the page terminated at the end of the comment header.
                 If there is another packet on the page, or part of a packet, then
                 reject the stream.
                 Otherwise seekable sources won't be able to seek back to the start
                 properly.*/
                ret = ogg_stream_packetout(&m_OggOpusFile->os, &op);
                return 0;
            }
        }
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int op_fetch_headers(OpusHead_t *_head, OpusTags_t *_tags, uint32_t **_serialnos,
        int *_nserialnos, int *_cserialnos, ogg_page *_og) {
    ogg_page og;
    int ret;
    if(!_og) {
        /*No need to clamp the boundary offset against m_OggOpusFile->end, as all errors
         become OP_ENOTFORMAT.*/
        if(op_get_next_page(&og, OP_ADV_OFFSET(m_OggOpusFile->offset,OP_CHUNK_SIZE))<0) {
            return OP_ENOTFORMAT;
        }
        _og = &og;
    }
    m_OggOpusFile->ready_state = OP_OPENED;
    ret = op_fetch_headers_impl(_head, _tags, _serialnos, _nserialnos, _cserialnos, _og);
    /*Revert back from OP_STREAMSET to OP_OPENED on failure, to prevent
     double-free of the tags in an unseekable stream.*/
    if(ret < 0) m_OggOpusFile->ready_state = OP_OPENED;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
/*Granule position manipulation routines.
  A granule position is defined to be an unsigned 64-bit integer, with the
   special value -1 in two's complement indicating an unset or invalid granule
   position.
  We are not guaranteed to have an unsigned 64-bit type, so we construct the
   following routines that
   a) Properly order negative numbers as larger than positive numbers, and
   b) Check for underflow or overflow past the special -1 value.
  This lets us operate on the full, valid range of granule positions in a
   consistent and safe manner.
  This full range is organized into distinct regions:
   [ -1 (invalid) ][ 0 ... INT64_MAX ][ INT64_MIN ... -2 ][-1 (invalid) ]

  No one should actually use granule positions so large that they're negative,
   even if they are technically valid, as very little software handles them
   correctly (including most of Xiph.Org's).
  This library also refuses to support durations so large they won't fit in a
   signed 64-bit integer (to avoid exposing this mess to the application, and
   to simplify a good deal of internal arithmetic), so the only way to use them
   successfully is if pcm_start is very large.
  This means there isn't anything you can do with negative granule positions
   that you couldn't have done with purely non-negative ones.
  The main purpose of these routines is to allow us to think very explicitly
   about the possible failure cases of all granule position manipulations.*/

/*Safely adds a small signed integer to a valid (not -1) granule position.
  The result can use the full 64-bit range of values (both positive and
   negative), but will fail on overflow (wrapping past -1; wrapping past
   INT64_MAX is explicitly okay).
  [out] _dst_gp: The resulting granule position.
                 Only modified on success.
  _src_gp:       The granule position to add to.
                 This must not be -1.
  _delta:        The amount to add.
                 This is allowed to be up to 32 bits to support the maximum
                  duration of a single Ogg page (255 packets * 120 ms per
                  packet == 1,468,800 samples at 48 kHz).
  Return: 0 on success, or OP_EINVAL if the result would wrap around past -1.*/
int op_granpos_add(int64_t *_dst_gp, int64_t _src_gp, int32_t _delta) {
    /*The code below handles this case correctly, but there's no reason we
     should ever be called with these values, so make sure we aren't.*/
    assert(_src_gp!=-1);
    if(_delta > 0) {
        /*Adding this amount to the granule position would overflow its 64-bit
         range.*/
        if((_src_gp<0) && (_src_gp >= -1 - _delta)) return OP_EINVAL;
        if(_src_gp>INT64_MAX-_delta) {
            /*Adding this amount to the granule position would overflow the positive
             half of its 64-bit range.
             Since signed overflow is undefined in C, do it in a way the compiler
             isn't allowed to screw up.*/
            _delta -= (int32_t) (INT64_MAX - _src_gp) + 1;
            _src_gp = INT64_MIN;
        }
    }
    else if(_delta < 0) {
        /*Subtracting this amount from the granule position would underflow its
         64-bit range.*/
        if(_src_gp >= 0 && (_src_gp < -_delta)) return OP_EINVAL;
        if(_src_gp<INT64_MIN-_delta) {
            /*Subtracting this amount from the granule position would underflow the
             negative half of its 64-bit range.
             Since signed underflow is undefined in C, do it in a way the compiler
             isn't allowed to screw up.*/
            _delta += (int32_t) (_src_gp - INT64_MIN) + 1;
            _src_gp = INT64_MAX;
        }
    }
    *_dst_gp = _src_gp + _delta;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Safely computes the difference between two granule positions.
  The difference must fit in a signed 64-bit integer, or the function fails.
  It correctly handles the case where the granule position has wrapped around
   from positive values to negative ones.
  [out] _delta: The difference between the granule positions.
                Only modified on success.
  _gp_a:        The granule position to subtract from.
                This must not be -1.
  _gp_b:        The granule position to subtract.
                This must not be -1.
  Return: 0 on success, or OP_EINVAL if the result would not fit in a signed
           64-bit integer.*/
int op_granpos_diff(int64_t *_delta, int64_t _gp_a, int64_t _gp_b) {
    int gp_a_negative;
    int gp_b_negative;
    /*The code below handles these cases correctly, but there's no reason we
     should ever be called with these values, so make sure we aren't.*/
    assert(_gp_a!=-1); assert(_gp_b!=-1);
    gp_a_negative = (_gp_a < 0);
    gp_b_negative = (_gp_b < 0);
    if(gp_a_negative ^ gp_b_negative) {
        int64_t da;
        int64_t db;
        if(gp_a_negative) {
            /*_gp_a has wrapped to a negative value but _gp_b hasn't: the difference
             should be positive.*/
            /*Step 1: Handle wrapping.*/
            /*_gp_a < 0 => da < 0.*/
            da = (INT64_MIN - _gp_a) - 1;
            /*_gp_b >= 0  => db >= 0.*/
            db = INT64_MAX - _gp_b;
            /*Step 2: Check for overflow.*/
            if(INT64_MAX+da<db) return OP_EINVAL;
            *_delta = db - da;
        }
        else {
            /*_gp_b has wrapped to a negative value but _gp_a hasn't: the difference
             should be negative.*/
            /*Step 1: Handle wrapping.*/
            /*_gp_a >= 0 => da <= 0*/
            da = _gp_a + INT64_MIN;
            /*_gp_b < 0 => db <= 0*/
            db = INT64_MIN - _gp_b;
            /*Step 2: Check for overflow.*/
            if(da<INT64_MIN-db) return OP_EINVAL;
            *_delta = da + db;
        }
    }
    else
        *_delta = _gp_a - _gp_b;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int op_granpos_cmp(int64_t _gp_a, int64_t _gp_b) {
    /*The invalid granule position -1 should behave like NaN: neither greater
     than nor less than any other granule position, nor equal to any other
     granule position, including itself.
     However, that means there isn't anything we could sensibly return from this
     function for it.*/
    assert(_gp_a!=-1); assert(_gp_b!=-1);
    /*Handle the wrapping cases.*/
    if(_gp_a < 0) {
        if(_gp_b >= 0) return 1;
        /*Else fall through.*/
    }
    else if(_gp_b < 0) return -1;
    /*No wrapping case.*/
    return (_gp_a > _gp_b) - (_gp_b > _gp_a);
}
//----------------------------------------------------------------------------------------------------------------------
/*Returns the duration of the packet (in samples at 48 kHz), or a negative
   value on error.*/

int op_get_packet_duration(uint8_t *_data, int _len) {
    int nframes;
    int frame_size;
    int nsamples;
    nframes = opus_packet_get_nb_frames(_data, _len);
    if(nframes < 0) return OP_EBADPACKET;
    frame_size = opus_packet_get_samples_per_frame(_data, 48000);
    nsamples = nframes * frame_size;
    if(nsamples > 120 * 48) return OP_EBADPACKET;
    return nsamples;
}
//----------------------------------------------------------------------------------------------------------------------

/*Grab all the packets currently in the stream state, and compute their
   durations.
  m_OggOpusFile->op_count is set to the number of packets collected.
  [out] _durations: Returns the durations of the individual packets.
  Return: The total duration of all packets, or OP_HOLE if there was a hole.*/
int32_t op_collect_audio_packets(int _durations[255]) {
    int32_t total_duration;
    int op_count;
    /*Count the durations of all packets in the page.*/
    op_count = 0;
    total_duration = 0;
    for(;;) {
        int ret;
        /*This takes advantage of undocumented libogg behavior that returned
         ogg_packet buffers are valid at least until the next page is
         submitted.
         Relying on this is not too terrible, as _none_ of the Ogg memory
         ownership/lifetime rules are well-documented.
         But I can read its code and know this will work.*/
        ret = ogg_stream_packetout(&m_OggOpusFile->os, m_OggOpusFile->op + op_count);
        if(!ret) break;
        if(ret < 0) {
            /*We shouldn't get holes in the middle of pages.*/
            assert(op_count==0);
            /*Set the return value and break out of the loop.
             We want to make sure op_count gets set to 0, because we've ingested a
             page, so any previously loaded packets are now invalid.*/
            total_duration = OP_HOLE;
            break;
        }
        /*Unless libogg is broken, we can't get more than 255 packets from a
         single page.*/
        assert(op_count<255);
        _durations[op_count] = op_get_packet_duration(m_OggOpusFile->op[op_count].packet, m_OggOpusFile->op[op_count].bytes);
        if(_durations[op_count] > 0) {
            /*With at most 255 packets on a page, this can't overflow.*/
            total_duration += _durations[op_count++];
        }
        /*Ignore packets with an invalid TOC sequence.*/
        else if(op_count > 0) {
            /*But save the granule position, if there was one.*/
            m_OggOpusFile->op[op_count - 1].granulepos = m_OggOpusFile->op[op_count].granulepos;
        }
    }
    m_OggOpusFile->op_pos = 0;
    m_OggOpusFile->op_count = op_count;
    return total_duration;
}
//----------------------------------------------------------------------------------------------------------------------
/*Starting from current cursor position, get the initial PCM offset of the next
   page.
  This also validates the granule position on the first page with a completed
   audio data packet, as required by the spec.
  If this link is completely empty (no pages with completed packets), then this
   function sets pcm_start=pcm_end=0 and returns the BOS page of the next link
   (if any).
  In the seekable case, we initialize pcm_end=-1 before calling this function,
   so that later we can detect that the link was empty before calling
   op_find_final_pcm_offset().
  [inout] _link: The link for which to find pcm_start.
  [out] _og:     Returns the BOS page of the next link if this link was empty.
                 In the unseekable case, we can then feed this to
                  op_fetch_headers() to start the next link.
                 The caller may pass NULL (e.g., for seekable streams), in
                  which case this page will be discarded.
  Return: 0 on success, 1 if there is a buffered BOS page available, or a
           negative value on unrecoverable error.*/
int op_find_initial_pcm_offset(ogg_page *_og) {
    ogg_page og;
    int64_t page_offset;
    int64_t pcm_start;
    int64_t prev_packet_gp;
    int64_t cur_page_gp;
    uint32_t serialno;
    int32_t total_duration;
    int *durations = (int*) malloc(255 * sizeof(int));
    int cur_page_eos;
    int op_count;
    int pi;
    if(_og == NULL) _og = &og;
    serialno = m_OggOpusFile->os.serialno;
    op_count = 0;
    /*We shouldn't have to initialize total_duration, but gcc is too dumb to
     figure out that op_count>0 implies we've been through the whole loop at
     least once.*/
    total_duration = 0;
    do {
        page_offset = op_get_next_page(_og, m_OggOpusFile->end);
        /*We should get a page unless the file is truncated or mangled.
         Otherwise there are no audio data packets in the whole logical stream.*/
        if(page_offset < 0) {
            /*Fail if there was a read error.*/
            if(page_offset < OP_FALSE) {
                free(durations);
                return (int) page_offset;
            }
            /*Fail if the pre-skip is non-zero, since it's asking us to skip more
             samples than exist.*/
            if(m_OggOpusLink->head.pre_skip > 0) {
                free(durations);
                return OP_EBADTIMESTAMP;
            }
            m_OggOpusLink->pcm_file_offset = 0;
            /*Set pcm_end and end_offset so we can skip the call to
             op_find_final_pcm_offset().*/
            m_OggOpusLink->pcm_start = m_OggOpusLink->pcm_end = 0;
            m_OggOpusLink->end_offset = m_OggOpusLink->data_offset;
            free(durations);
            return 0;
        }
        /*Similarly, if we hit the next link in the chain, we've gone too far.*/
        if(ogg_page_bos(_og)) {
            if(m_OggOpusLink->head.pre_skip > 0) {
                free(durations);
                return OP_EBADTIMESTAMP;
            }
            /*Set pcm_end and end_offset so we can skip the call to
             op_find_final_pcm_offset().*/
            m_OggOpusLink->pcm_file_offset = 0;
            m_OggOpusLink->pcm_start = m_OggOpusLink->pcm_end = 0;
            m_OggOpusLink->end_offset = m_OggOpusLink->data_offset;
            /*Tell the caller we've got a buffered page for them.*/
            free(durations);
            return 1;
        }
        /*Ignore pages from other streams (not strictly necessary, because of the
         checks in ogg_stream_pagein(), but saves some work).*/
        if(serialno != (uint32_t) getSerialNo(_og)) continue;
        ogg_stream_pagein(&m_OggOpusFile->os, _og);
        /*Bitrate tracking: add the header's bytes here.
         The body bytes are counted when we consume the packets.*/
        m_OggOpusFile->bytes_tracked += _og->header_len;
        /*Count the durations of all packets in the page.*/
        do
            total_duration = op_collect_audio_packets(durations);
        /*Ignore holes.*/
        while(total_duration < 0);
        op_count = m_OggOpusFile->op_count;
    } while(op_count <= 0);
    /*We found the first page with a completed audio data packet: actually look
     at the granule position.
     RFC 3533 says, "A special value of -1 (in two's complement) indicates that
     no packets finish on this page," which does not say that a granule
     position that is NOT -1 indicates that some packets DO finish on that page
     (even though this was the intention, libogg itself violated this intention
     for years before we fixed it).
     The Ogg Opus specification only imposes its start-time requirements
     on the granule position of the first page with completed packets,
     so we ignore any set granule positions until then.*/
    cur_page_gp = m_OggOpusFile->op[op_count - 1].granulepos;
    /*But getting a packet without a valid granule position on the page is not
     okay.*/
    if(cur_page_gp == -1) {
        free(durations);
        return OP_EBADTIMESTAMP;
    }
    cur_page_eos = m_OggOpusFile->op[op_count - 1].e_o_s;
    if(!cur_page_eos) {
        /*The EOS flag wasn't set.
         Work backwards from the provided granule position to get the starting PCM
         offset.*/
        if(op_granpos_add(&pcm_start, cur_page_gp, -total_duration) < 0) {
            /*The starting granule position MUST not be smaller than the amount of
             audio on the first page with completed packets.*/
            free(durations);
            return OP_EBADTIMESTAMP;
        }
    }
    else {
        /*The first page with completed packets was also the last.*/
        if(op_granpos_add(&pcm_start, cur_page_gp, -total_duration) < 0) {
            /*If there's less audio on the page than indicated by the granule
             position, then we're doing end-trimming, and the starting PCM offset
             is zero by spec mandate.*/
            pcm_start = 0;
            /*However, the end-trimming MUST not ask us to trim more samples than
             exist after applying the pre-skip.*/
            if(op_granpos_cmp(cur_page_gp, m_OggOpusLink->head.pre_skip) < 0) {
                free(durations);
                return OP_EBADTIMESTAMP;
            }
        }
    }
    /*Timestamp the individual packets.*/
    prev_packet_gp = pcm_start;
    for(pi = 0; pi < op_count; pi++) {
        if(cur_page_eos) {
            int64_t diff;
            op_granpos_diff(&diff, cur_page_gp, prev_packet_gp);
            diff = durations[pi] - diff;
            /*If we have samples to trim...*/
            if(diff > 0) {
                /*If we trimmed the entire packet, stop (the spec says encoders
                 shouldn't do this, but we support it anyway).*/
                if(diff > durations[pi]) break;
                m_OggOpusFile->op[pi].granulepos = prev_packet_gp = cur_page_gp;
                /*Move the EOS flag to this packet, if necessary, so we'll trim the
                 samples.*/
                m_OggOpusFile->op[pi].e_o_s = 1;
                continue;
            }
        }
        /*Update the granule position as normal.*/
        op_granpos_add(&m_OggOpusFile->op[pi].granulepos, prev_packet_gp, durations[pi]);
        prev_packet_gp = m_OggOpusFile->op[pi].granulepos;
    }
    /*Update the packet count after end-trimming.*/
    m_OggOpusFile->op_count = pi;
    m_OggOpusFile->cur_discard_count = m_OggOpusLink->head.pre_skip;
    m_OggOpusLink->pcm_file_offset = 0;
    m_OggOpusFile->prev_packet_gp = m_OggOpusLink->pcm_start = pcm_start;
    m_OggOpusFile->prev_page_offset = page_offset;
    free(durations);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Rescale the number _x from the range [0,_from] to [0,_to]. _from and _to must be positive.*/
int64_t op_rescale64(int64_t _x, int64_t _from, int64_t _to) {
    int64_t frac;
    int64_t ret;
    int i;
    if(_x >= _from) return _to;
    if(_x <= 0) return 0;
    frac = 0;
    for(i = 0; i < 63; i++) {
        frac <<= 1;
        assert(_x<=_from);
        if(_x >= _from >> 1) {
            _x -= _from - _x;
            frac |= 1;
        }
        else
            _x <<= 1;
    }
    ret = 0;
    for(i = 0; i < 63; i++) {
        if(frac & 1)
            ret = (ret & _to & 1) + (ret >> 1) + (_to >> 1);
        else
            ret >>= 1;
        frac >>= 1;
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

/*The minimum granule position spacing allowed for making predictions.
  This corresponds to about 1 second of audio at 48 kHz for both Opus and
   Vorbis, or one keyframe interval in Theora with the default keyframe spacing
   of 256.*/
#define OP_GP_SPACING_MIN (48000)
//----------------------------------------------------------------------------------------------------------------------
int op_make_decode_ready() {
    const OpusHead_t *head;
    int li;
    int stream_count;
    int coupled_count;
    int channel_count;
    if(m_OggOpusFile->ready_state > OP_STREAMSET) return 0;
    if(m_OggOpusFile->ready_state < OP_STREAMSET) return OP_EFAULT;
    li = m_OggOpusFile->seekable ? m_OggOpusFile->cur_link : 0;
    head = &m_OggOpusLink[li].head;
    stream_count = head->stream_count;
    coupled_count = head->coupled_count;
    channel_count = head->channel_count;
    /*Check to see if the current decoder is compatible with the current link.*/
    if(m_od != NULL && m_OggOpusFile->od_stream_count == stream_count && m_OggOpusFile->od_coupled_count == coupled_count
            && m_OggOpusFile->od_channel_count == channel_count
            && memcmp(m_OggOpusFile->od_mapping, head->mapping, sizeof(*head->mapping) * channel_count) == 0) {
        opus_multistream_decoder_ctl(m_od, OPUS_RESET_STATE);
    }
    else {
        int err;
        opus_multistream_decoder_destroy(m_od);
        m_od = opus_multistream_decoder_create(48000, channel_count, stream_count, coupled_count, head->mapping,
                &err);
        if(m_od == NULL) return OP_EFAULT;
        m_OggOpusFile->od_stream_count = stream_count;
        m_OggOpusFile->od_coupled_count = coupled_count;
        m_OggOpusFile->od_channel_count = channel_count;
        memcpy(m_OggOpusFile->od_mapping, head->mapping, sizeof(*head->mapping) * channel_count);
    }
    m_OggOpusFile->ready_state = OP_INITSET;
    m_OggOpusFile->bytes_tracked = 0;
    m_OggOpusFile->samples_tracked = 0;
    //op_update_gain(m_OggOpusFile);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Clear out the current logical bitstream decoder.*/
void op_decode_clear() {
    /*We don't actually free the decoder.
     We might be able to re-use it for the next link.*/
    m_OggOpusFile->op_count = 0;
    m_OggOpusFile->od_buffer_size = 0;
    m_OggOpusFile->prev_packet_gp = -1;
    m_OggOpusFile->prev_page_offset = -1;
    m_OggOpusFile->ready_state = OP_OPENED;
}
//----------------------------------------------------------------------------------------------------------------------
void op_clear() {
    OggOpusLink_t *links;
    free(m_OggOpusFile->od_buffer);
    if(m_od != NULL) opus_multistream_decoder_destroy(m_od);
    links = m_OggOpusLink;
    free(links);
    free(m_OggOpusFile->serialnos);
    ogg_stream_clear(&m_OggOpusFile->os);
    ogg_sync_clear(&m_OggOpusFile->oy);
}
//----------------------------------------------------------------------------------------------------------------------
int op_open1() {
    ogg_page og;
    ogg_page *pog;
    int seekable = 0;
    int ret;
    memset(m_OggOpusFile, 0, sizeof(*m_OggOpusFile));
    m_OggOpusFile->end = -1;
    m_OggOpusFile->oy.storage = -1;
    memset(&m_OggOpusFile->oy, 0, sizeof(m_OggOpusFile->oy));

    /*Don't seek yet.
     Set up a 'single' (current) logical bitstream entry for partial open.*/
    m_OggOpusLink = (OggOpusLink_t*) malloc(sizeof(*m_OggOpusLink));
    /*The serialno gets filled in later by op_fetch_headers().*/
    ogg_stream_init(&m_OggOpusFile->os, -1);
    pog = NULL;
    for(;;) {
        /*Fetch all BOS pages, store the Opus header and all seen serial numbers,
         and load subsequent Opus setup headers.*/
        ret = op_fetch_headers(&m_OggOpusLink[0].head, &m_OggOpusLink[0].tags, &m_OggOpusFile->serialnos,
                               &m_OggOpusFile->nserialnos, &m_OggOpusFile->cserialnos, pog);
        if(ret < 0) break;
        m_OggOpusFile->nlinks = 1;
        m_OggOpusLink[0].offset = 0;
        m_OggOpusLink[0].data_offset = m_OggOpusFile->offset;
        m_OggOpusLink[0].pcm_end = -1;
        m_OggOpusLink[0].serialno = m_OggOpusFile->os.serialno;
        /*Fetch the initial PCM offset.*/
        ret = op_find_initial_pcm_offset(&og);
        if(seekable || (ret <= 0)) break;
        m_OggOpusFile->nlinks = 0;
        if(!seekable) m_OggOpusFile->cur_link++;
        pog = &og;
    }
    if(ret >= 0) m_OggOpusFile->ready_state = OP_PARTOPEN;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
int op_open2() {
    log_i("op_open2");
    int ret;
    assert(m_OggOpusFile->ready_state==OP_PARTOPEN);
    ret = 0;
    if(ret >= 0) {
        /*We have buffered packets from op_find_initial_pcm_offset().
         Move to OP_INITSET so we can use them.*/
        m_OggOpusFile->ready_state = OP_STREAMSET;
        ret = op_make_decode_ready();
        if(ret >= 0) return 0;
    }
    op_clear();
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
OggOpusFile_t* opus_init_decoder() {
    int ret = OP_EFAULT;
    SILKDecoder_AllocateBuffers();
    CELTDecoder_AllocateBuffers();
    m_OggOpusFile = (OggOpusFile_t*) malloc(sizeof(*m_OggOpusFile));
    if(m_OggOpusFile==NULL) return NULL;
    ret = op_open1();
    if(ret < 0) {
        op_clear();
        free(m_OggOpusFile);
    }
    ret = op_open2();
    if(ret >= 0) return m_OggOpusFile;

    free(m_OggOpusFile);
    return NULL;
}
//----------------------------------------------------------------------------------------------------------------------
/*Given a serialno, find a link with a corresponding Opus stream, if it exists.
 Return: The index of the link to which the page belongs, or a negative number
 if it was not a desired Opus bitstream section.*/
int op_get_link_from_serialno(int _cur_link, int64_t _page_offset,
        uint32_t _serialno) {
    const OggOpusLink_t *links;
    int nlinks;
    int li_lo;
    int li_hi;
    assert(m_OggOpusFile->seekable);
    links = m_OggOpusLink;
    nlinks = m_OggOpusFile->nlinks;
    li_lo = 0;
    /*Start off by guessing we're just a multiplexed page in the current link.*/
    li_hi = _cur_link + 1 < nlinks && _page_offset < links[_cur_link + 1].offset ? _cur_link + 1 : nlinks;
    do {
        if(_page_offset >= links[_cur_link].offset)
            li_lo = _cur_link;
        else
            li_hi = _cur_link;
        _cur_link = ((li_lo + (li_hi - li_lo)) >> 1);
    } while(li_hi - li_lo > 1);
    /*We've identified the link that should contain this page.
     Make sure it's a page we care about.*/
    if(links[_cur_link].serialno != _serialno) return OP_FALSE;
    return _cur_link;
}
//----------------------------------------------------------------------------------------------------------------------
/*Fetch and process a page. This handles the case where we're at a bitstream boundary and dumps the decoding machine.
 If the decoding machine is unloaded, it loads it.
 It also keeps prev_packet_gp up to date (seek and read both use this).
 Return: <0) Error, OP_HOLE (lost packet), or OP_EOF.
 0) Got at least one audio data packet.*/
int op_fetch_and_process_page(ogg_page *_og, int64_t _page_offset, int _spanp,
        int _ignore_holes) {
    OggOpusLink_t *links;
    uint32_t cur_serialno;
    int seekable;
    int cur_link;
    int ret;

    /*We shouldn't get here if we have unprocessed packets.*/
    assert(m_OggOpusFile->ready_state<OP_INITSET||m_OggOpusFile->op_pos>=m_OggOpusFile->op_count);

    seekable = m_OggOpusFile->seekable;
    links = m_OggOpusLink;
    cur_link = seekable ? m_OggOpusFile->cur_link : 0;
    cur_serialno = links[cur_link].serialno;
    /*Handle one page.*/
    for(;;) {
        ogg_page og;
        assert(m_OggOpusFile->ready_state>=OP_OPENED);
        /*If we were given a page to use, use it.*/
        if(_og != NULL) {
            *&og = *_og;
            _og = NULL;
        }
        /*Keep reading until we get a page with the correct serialno.*/
        else
            _page_offset = op_get_next_page( &og, m_OggOpusFile->end);
        /*EOF: Leave uninitialized.*/
        if(_page_offset < 0) return _page_offset < OP_FALSE ? (int) _page_offset : OP_EOF;
        if((m_OggOpusFile->ready_state>=OP_STREAMSET) && cur_serialno != (uint32_t) getSerialNo(&og)) {
            /*Two possibilities:
             1) Another stream is multiplexed into this logical section, or*/
            if(!ogg_page_bos(&og)) continue;
            /* 2) Our decoding just traversed a bitstream boundary.*/
            if(!_spanp) return OP_EOF;
            if(m_OggOpusFile->ready_state>=OP_INITSET) op_decode_clear();
        }
        /*Bitrate tracking: add the header's bytes here.
         The body bytes are counted when we consume the packets.*/
        else
            m_OggOpusFile->bytes_tracked += og.header_len;
        /*Do we need to load a new machine before submitting the page?
         This is different in the seekable and non-seekable cases.
         In the seekable case, we already have all the header information loaded
         and cached.
         We just initialize the machine with it and continue on our merry way.
         In the non-seekable (streaming) case, we'll only be at a boundary if we
         just left the previous logical bitstream, and we're now nominally at the
         header of the next bitstream.*/
        if(m_OggOpusFile->ready_state<OP_STREAMSET) {
            if(seekable) {
                uint32_t serialno;
                serialno = getSerialNo(&og);
                /*Match the serialno to bitstream section.*/
                assert(cur_link>=0&&cur_link<m_OggOpusFile->nlinks);
                if(links[cur_link].serialno != serialno) {
                    /*It wasn't a page from the current link.
                     Is it from the next one?*/
                    if((cur_link + 1 < m_OggOpusFile->nlinks && links[cur_link + 1].serialno == serialno)) {
                        cur_link++;
                    }
                    else {
                        int new_link;
                        new_link = op_get_link_from_serialno( cur_link, _page_offset, serialno);
                        /*Not a desired Opus bitstream section.
                         Keep trying.*/
                        if(new_link < 0) continue;
                        cur_link = new_link;
                    }
                }
                cur_serialno = serialno;
                m_OggOpusFile->cur_link = cur_link;
                ogg_stream_reset_serialno(&m_OggOpusFile->os, serialno);
                m_OggOpusFile->ready_state = OP_STREAMSET;
                /*If we're at the start of this link, initialize the granule position
                 and pre-skip tracking.*/
                if(_page_offset <= links[cur_link].data_offset) {
                    m_OggOpusFile->prev_packet_gp = links[cur_link].pcm_start;
                    m_OggOpusFile->prev_page_offset = -1;
                    m_OggOpusFile->cur_discard_count = links[cur_link].head.pre_skip;
                    /*Ignore a hole at the start of a new link (this is common for
                     streams joined in the middle) or after seeking.*/
                    _ignore_holes = 1;
                }
            }
            else {
                do {
                    /*We're streaming.
                     Fetch the two header packets, build the info struct.*/
                    ret = op_fetch_headers( &links[0].head, &links[0].tags,
                    NULL, NULL, NULL, &og);
                    if(ret < 0) return ret;
                    /*op_find_initial_pcm_offset() will suppress any initial hole for us,
                     so no need to set _ignore_holes.*/
                    ret = op_find_initial_pcm_offset(&og);
                    if(ret < 0) return ret;
                    m_OggOpusLink[0].serialno = cur_serialno = m_OggOpusFile->os.serialno;
                    m_OggOpusFile->cur_link++;
                }
                /*If the link was empty, keep going, because we already have the
                 BOS page of the next one in og.*/
                while(ret > 0);
                /*If we didn't get any packets out of op_find_initial_pcm_offset(),
                 keep going (this is possible if end-trimming trimmed them all).*/
                if(m_OggOpusFile->op_count <= 0) continue;
                /*Otherwise, we're done.
                 TODO: This resets bytes_tracked, which misses the header bytes
                 already processed by op_find_initial_pcm_offset().*/
                ret = op_make_decode_ready();
                if(ret < 0) return ret;
                return 0;
            }
        }
        /*The buffered page is the data we want, and we're ready for it.
         Add it to the stream state.*/
        if(m_OggOpusFile->ready_state==OP_STREAMSET) {
            ret = op_make_decode_ready();
            if(ret < 0) return ret;
        }
        /*Extract all the packets from the current page.*/
        ogg_stream_pagein(&m_OggOpusFile->os, &og);
        if(m_OggOpusFile->ready_state>=OP_INITSET) {
            int32_t total_duration;
            int *durations = (int*) malloc(255 * sizeof(int));
            int op_count;
            int report_hole;
            report_hole = 0;
            total_duration = op_collect_audio_packets(durations);
            if(total_duration < 0) {
                /*libogg reported a hole (a gap in the page sequence numbers).
                 Drain the packets from the page anyway.
                 If we don't, they'll still be there when we fetch the next page.
                 Then, when we go to pull out packets, we might get more than 255,
                 which would overrun our packet buffer.
                 We repeat this call until we get any actual packets, since we might
                 have buffered multiple out-of-sequence pages with no packets on
                 them.*/
                do
                    total_duration = op_collect_audio_packets(durations);
                while(total_duration < 0);
                if(!_ignore_holes) {
                    /*Report the hole to the caller after we finish timestamping the
                     packets.*/
                    report_hole = 1;
                    /*We had lost or damaged pages, so reset our granule position
                     tracking.
                     This makes holes behave the same as a small raw seek.
                     If the next page is the EOS page, we'll discard it (because we
                     can't perform end trimming properly), and we'll always discard at
                     least 80 ms of audio (to allow decoder state to re-converge).
                     We could try to fill in the gap with PLC by looking at timestamps
                     in the non-EOS case, but that's complicated and error prone and we
                     can't rely on the timestamps being valid.*/
                    m_OggOpusFile->prev_packet_gp = -1;
                }
            }
            op_count = m_OggOpusFile->op_count;
            /*If we found at least one audio data packet, compute per-packet granule
             positions for them.*/
            if(op_count > 0) {
                int64_t diff;
                int64_t prev_packet_gp;
                int64_t cur_packet_gp;
                int64_t cur_page_gp;
                int cur_page_eos;
                int pi;
                cur_page_gp = m_OggOpusFile->op[op_count - 1].granulepos;
                cur_page_eos = m_OggOpusFile->op[op_count - 1].e_o_s;
                prev_packet_gp = m_OggOpusFile->prev_packet_gp;
                if(prev_packet_gp == -1) {
                    int32_t cur_discard_count;
                    /*This is the first call after a raw seek.
                     Try to reconstruct prev_packet_gp from scratch.*/
                    assert(seekable);
                    if(cur_page_eos) {
                        /*If the first page we hit after our seek was the EOS page, and
                         we didn't start from data_offset or before, we don't have
                         enough information to do end-trimming.
                         Proceed to the next link, rather than risk playing back some
                         samples that shouldn't have been played.*/
                        m_OggOpusFile->op_count = 0;
                        if(report_hole) {
                            free(durations);
                            return OP_HOLE;
                        }
                        continue;
                    }
                    /*By default discard 80 ms of data after a seek, unless we seek
                     into the pre-skip region.*/
                    cur_discard_count = 80 * 48;
                    cur_page_gp = m_OggOpusFile->op[op_count - 1].granulepos;
                    /*Try to initialize prev_packet_gp.
                     If the current page had packets but didn't have a granule
                     position, or the granule position it had was too small (both
                     illegal), just use the starting granule position for the link.*/
                    prev_packet_gp = links[cur_link].pcm_start;
                    if(cur_page_gp != -1) {
                        op_granpos_add(&prev_packet_gp, cur_page_gp, -total_duration);
                    }
                    if(!op_granpos_diff(&diff, prev_packet_gp, links[cur_link].pcm_start)) {
                        int32_t pre_skip;
                        /*If we start at the beginning of the pre-skip region, or we're
                         at least 80 ms from the end of the pre-skip region, we discard
                         to the end of the pre-skip region.
                         Otherwise, we still use the 80 ms default, which will discard
                         past the end of the pre-skip region.*/
                        pre_skip = links[cur_link].head.pre_skip;
                        if(diff >= 0 && diff <= _max(0, pre_skip - 80 * 48)) {
                            cur_discard_count = pre_skip - (int) diff;
                        }
                    }
                    m_OggOpusFile->cur_discard_count = cur_discard_count;
                }
                if(cur_page_gp == -1) {
                    /*This page had completed packets but didn't have a valid granule
                     position.
                     This is illegal, but we'll try to handle it by continuing to count
                     forwards from the previous page.*/
                    if(op_granpos_add(&cur_page_gp, prev_packet_gp, total_duration) < 0) {
                        /*The timestamp for this page overflowed.*/
                        cur_page_gp = links[cur_link].pcm_end;
                    }
                }
                /*If we hit the last page, handle end-trimming.*/
                if((cur_page_eos) &&(!op_granpos_diff(&diff,cur_page_gp,prev_packet_gp))
                &&(diff<total_duration)) {
                    cur_packet_gp = prev_packet_gp;
                    for(pi = 0; pi < op_count; pi++) {
                        /*Check for overflow.*/
                        if(diff < 0 && (INT64_MAX+diff<durations[pi])) {
                            diff = durations[pi] + 1;
                        }
                        else
                            diff = durations[pi] - diff;
                        /*If we have samples to trim...*/
                        if(diff > 0) {
                            /*If we trimmed the entire packet, stop (the spec says encoders
                             shouldn't do this, but we support it anyway).*/
                            if(diff > durations[pi]) break;
                            cur_packet_gp = cur_page_gp;
                            /*Move the EOS flag to this packet, if necessary, so we'll trim
                             the samples during decode.*/
                            m_OggOpusFile->op[pi].e_o_s = 1;
                        }
                        else {
                            /*Update the granule position as normal.*/
                            op_granpos_add(&cur_packet_gp, cur_packet_gp, durations[pi]);
                        }
                        m_OggOpusFile->op[pi].granulepos = cur_packet_gp;
                        op_granpos_diff(&diff, cur_page_gp, cur_packet_gp);
                    }
                }
                else {
                    /*Propagate timestamps to earlier packets.
                     op_granpos_add(&prev_packet_gp,prev_packet_gp,total_duration)
                     should succeed and give prev_packet_gp==cur_page_gp.
                     But we don't bother to check that, as there isn't much we can do
                     if it's not true, and it actually will not be true on the first
                     page after a seek, if there was a continued packet.
                     The only thing we guarantee is that the start and end granule
                     positions of the packets are valid, and that they are monotonic
                     within a page.
                     They might be completely out of range for this link (we'll check
                     that elsewhere), or non-monotonic between pages.*/
                    if(op_granpos_add(&prev_packet_gp, cur_page_gp, -total_duration) < 0) {
                        /*The starting timestamp for the first packet on this page
                         underflowed.
                         This is illegal, but we ignore it.*/
                        prev_packet_gp = 0;
                    }
                    for(pi = 0; pi < op_count; pi++) {
                        if(op_granpos_add(&cur_packet_gp, cur_page_gp, -total_duration) < 0) {
                            /*The start timestamp for this packet underflowed.
                             This is illegal, but we ignore it.*/
                            cur_packet_gp = 0;
                        }
                        total_duration -= durations[pi];
                        assert(total_duration>=0);
                        op_granpos_add(&cur_packet_gp, cur_packet_gp, durations[pi]);
                        m_OggOpusFile->op[pi].granulepos = cur_packet_gp;
                    }assert(total_duration==0);
                }
                m_OggOpusFile->prev_packet_gp = prev_packet_gp;
                m_OggOpusFile->prev_page_offset = _page_offset;
                m_OggOpusFile->op_count = op_count = pi;
            }
            if(report_hole) {
                free(durations);
                return OP_HOLE;
            }
            /*If end-trimming didn't trim all the packets, we're done.*/
            if(op_count > 0) {
                free(durations);
                return 0;
            }
        }
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*A small helper to determine if an Ogg page contains data that continues onto
 a subsequent page.*/
int op_page_continues(const ogg_page *_og) {
    int nlacing;
    assert(_og->header_len>=27);
    nlacing = _og->header[26];
    assert(_og->header_len>=27+nlacing);
    /*This also correctly handles the (unlikely) case of nlacing==0, because
     0!=255.*/
    return _og->header[27 + nlacing - 1] == 255;
}
//----------------------------------------------------------------------------------------------------------------------
/*Allocate the decoder scratch buffer.
 This is done lazily, since if the user provides large enough buffers, we'll
 never need it.*/
int op_init_buffer() {
    int nchannels_max;
    if(m_OggOpusFile->seekable) {
        const OggOpusLink_t *links;
        int nlinks;
        int li;
        links = m_OggOpusLink;
        nlinks = m_OggOpusFile->nlinks;
        nchannels_max = 1;
        for(li = 0; li < nlinks; li++) {
            nchannels_max = _max(nchannels_max, links[li].head.channel_count);
        }
    }
    else
        nchannels_max = OP_NCHANNELS_MAX;
    m_OggOpusFile->od_buffer = (int16_t*) malloc(sizeof(*m_OggOpusFile->od_buffer) * nchannels_max * 120 * 48);
    if(m_OggOpusFile->od_buffer == NULL) return OP_EFAULT;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Read more samples from the stream, using the same API as op_read() or op_read_float().*/
int op_read_native() {

    int16_t *_pcm = NULL;
    int _buf_size = 0;
    int *_li = NULL;

    if(m_OggOpusFile->ready_state<OP_OPENED) return OP_EINVAL;
    for(;;) {
        int ret;
        if(m_OggOpusFile->ready_state>=OP_INITSET) {
            int nchannels;
            int od_buffer_pos;
            int nsamples;
            int op_pos;
            nchannels = m_OggOpusLink[m_OggOpusFile->seekable ? m_OggOpusFile->cur_link : 0].head.channel_count;
            od_buffer_pos = m_OggOpusFile->od_buffer_pos;
            nsamples = m_OggOpusFile->od_buffer_size - od_buffer_pos;
            /*If we have buffered samples, return them.*/
            if(nsamples > 0) {
                if(nsamples * nchannels > _buf_size) nsamples = _buf_size / nchannels;
                assert(_pcm!=NULL||nsamples<=0);
                /*Check nsamples again so we don't pass NULL to memcpy() if _buf_size
                 is zero.
                 That would technically be undefined behavior, even if the number of
                 bytes to copy were zero.*/
                if(nsamples > 0) {
                    memcpy(_pcm, m_OggOpusFile->od_buffer + nchannels * od_buffer_pos, sizeof(*_pcm) * nchannels * nsamples);
                    od_buffer_pos += nsamples;
                    m_OggOpusFile->od_buffer_pos = od_buffer_pos;
                }
                if(_li != NULL) *_li = m_OggOpusFile->cur_link;
                return nsamples;
            }
            /*If we have buffered packets, decode one.*/
            op_pos = m_OggOpusFile->op_pos;
            if(op_pos < m_OggOpusFile->op_count) {
                const ogg_packet *pop;
                int64_t diff;
                int32_t cur_discard_count;
                int duration;
                int trimmed_duration;
                pop = m_OggOpusFile->op + op_pos++;
                m_OggOpusFile->op_pos = op_pos;
                cur_discard_count = m_OggOpusFile->cur_discard_count;
                duration = op_get_packet_duration(pop->packet, pop->bytes);
                /*We don't buffer packets with an invalid TOC sequence.*/
                assert(duration>0);
                trimmed_duration = duration;
                /*Perform end-trimming.*/
                if(pop->e_o_s) {
                    if(op_granpos_cmp(pop->granulepos, m_OggOpusFile->prev_packet_gp) <= 0) {
                        trimmed_duration = 0;
                    }
                    else if(!op_granpos_diff(&diff, pop->granulepos, m_OggOpusFile->prev_packet_gp)) {
                        trimmed_duration = (int) _min(diff, trimmed_duration);
                    }
                }
                m_OggOpusFile->prev_packet_gp = pop->granulepos;
                if(duration * nchannels > _buf_size) {
                    int16_t *buf;
                    /*If the user's buffer is too small, decode into a scratch buffer.*/
                    buf = m_OggOpusFile->od_buffer;
                    if(buf==NULL) {
                        ret = op_init_buffer();
                        if(ret < 0) return ret;
                        buf = m_OggOpusFile->od_buffer;
                    }
                    ret = opus_multistream_decode(m_od, pop->packet, pop->bytes, buf, duration);
                    if(ret < 0) return OP_EBADPACKET;
                    //if(ret < 0) return ret;
                    /*Perform pre-skip/pre-roll.*/
                    od_buffer_pos = (int) _min(trimmed_duration, cur_discard_count);
                    cur_discard_count -= od_buffer_pos;
                    m_OggOpusFile->cur_discard_count = cur_discard_count;
                    m_OggOpusFile->od_buffer_pos = od_buffer_pos;
                    m_OggOpusFile->od_buffer_size = trimmed_duration;
                    /*Update bitrate tracking based on the actual samples we used from
                     what was decoded.*/
                    m_OggOpusFile->bytes_tracked += pop->bytes;
                    m_OggOpusFile->samples_tracked += trimmed_duration - od_buffer_pos;
                }
                else {
                    assert(_pcm!=NULL);
                    /*Otherwise decode directly into the user's buffer.*/
                    ret = opus_multistream_decode(m_od, pop->packet, pop->bytes, _pcm, duration);
                    if(ret < 0) return OP_EBADPACKET;
                    if(trimmed_duration > 0) {
                        /*Perform pre-skip/pre-roll.*/
                        od_buffer_pos = (int) _min(trimmed_duration, cur_discard_count);
                        cur_discard_count -= od_buffer_pos;
                        m_OggOpusFile->cur_discard_count = cur_discard_count;
                        trimmed_duration -= od_buffer_pos;
                        if((trimmed_duration>0) && (od_buffer_pos > 0)) {
                            memmove(_pcm, _pcm + od_buffer_pos * nchannels,
                                    sizeof(*_pcm) * trimmed_duration * nchannels);
                        }
                        /*Update bitrate tracking based on the actual samples we used from
                         what was decoded.*/
                        m_OggOpusFile->bytes_tracked += pop->bytes;
                        m_OggOpusFile->samples_tracked += trimmed_duration;
                        if(trimmed_duration > 0) {
                            if(_li != NULL) *_li = m_OggOpusFile->cur_link;
                            return trimmed_duration;
                        }
                    }
                }
                /*Don't grab another page yet.
                 This one might have more packets, or might have buffered data now.*/
                continue;
            }
        }
        /*Suck in another page.*/
        ret = op_fetch_and_process_page(NULL, -1, 1, 0);
        if(ret==OP_EOF) {
            if(_li != NULL) *_li = m_OggOpusFile->cur_link;
            return 0;
        }
        if(ret < 0) return ret;
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int op_read_stereo(int16_t *_pcm, int _buf_size) {

    int ret;
    /*Ensure we have some decoded samples in our buffer.*/
    ret = op_read_native();
    /*Now apply the filter to them.*/
    if((ret>=0) && (m_OggOpusFile->ready_state>=OP_INITSET)) {
        int od_buffer_pos;
        od_buffer_pos = m_OggOpusFile->od_buffer_pos;
        ret = m_OggOpusFile->od_buffer_size - od_buffer_pos;
        if(ret > 0) {
            int nchannels;
            nchannels = m_OggOpusLink[m_OggOpusFile->seekable ? m_OggOpusFile->cur_link : 0].head.channel_count;

            int16_t *_src = m_OggOpusFile->od_buffer + nchannels * od_buffer_pos;

            ret = _min(ret, _buf_size >> 1);
            if(nchannels == 2)
                memcpy(_pcm, _src, ret * 2 * sizeof(*_src));
            else {
                int16_t *dst;
                int i;
                dst = (int16_t*) _pcm;
                if(nchannels == 1) {
                    for(i = 0; i < ret; i++)
                        dst[2 * i + 0] = dst[2 * i + 1] = _src[i];
                }
                else {
                    // noop, removed for RAM savings
                }
            }

            assert(ret>=0); assert(ret<=m_OggOpusFile->od_buffer_size-od_buffer_pos);
            od_buffer_pos += ret;
            m_OggOpusFile->od_buffer_pos = od_buffer_pos;
        }
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_head_parse(OpusHead_t *_head, uint8_t *_data, size_t _len) {
    OpusHead_t head;
    if(_len < 8) return OP_ENOTFORMAT;
    if(memcmp(_data, "OpusHead", 8) != 0) return OP_ENOTFORMAT;
    if(_len < 9) return OP_EBADHEADER;
    head.version = _data[8];
    if(head.version > 15) return OP_EVERSION;
    if(_len < 19) return OP_EBADHEADER;
    head.channel_count = _data[9];
    head.pre_skip = _data[10] | _data[11] << 8;
    head.input_sample_rate = _data[12] | (uint32_t) _data[13] << 8 | (uint32_t) _data[14] << 16 | (uint32_t) _data[15] << 24;

    int ret = _data[16] | _data[17] << 8;
    head.output_gain = (ret ^ 0x8000) - 0x8000;

    head.mapping_family = _data[18];
    if(head.mapping_family == 0) {
        if(head.channel_count < 1 || head.channel_count > 2) return OP_EBADHEADER;
        if(head.version <= 1 && _len > 19) return OP_EBADHEADER;
        head.stream_count = 1;
        head.coupled_count = head.channel_count - 1;
        if(_head != NULL) {
            _head->mapping[0] = 0;
            _head->mapping[1] = 1;
        }
    }
    else if(head.mapping_family == 1) {
        size_t size;
        int ci;
        if(head.channel_count < 1 || head.channel_count > 8) return OP_EBADHEADER;
        size = 21 + head.channel_count;
        if((_len < size) || ((head.version <= 1) && (_len > size))) return OP_EBADHEADER;
        head.stream_count = _data[19];
        if(head.stream_count < 1) return OP_EBADHEADER;
        head.coupled_count = _data[20];
        if(head.coupled_count > head.stream_count) return OP_EBADHEADER;
        for(ci = 0; ci < head.channel_count; ci++) {
            if(_data[21 + ci] >= head.stream_count + head.coupled_count && _data[21 + ci] != 255) {
                return OP_EBADHEADER;
            }
        }
        if(_head != NULL) memcpy(_head->mapping, _data + 21, head.channel_count);
    }
    /*General purpose players should not attempt to play back content with
     channel mapping family 255.*/
    else if(head.mapping_family == 255)
        return OP_EIMPL;
    /*No other channel mapping families are currently defined.*/
    else
        return OP_EBADHEADER;
    if(_head != NULL) memcpy(_head, &head, head.mapping - (unsigned char*) &head);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
