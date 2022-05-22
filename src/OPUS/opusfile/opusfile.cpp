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

//#ifdef HAVE_CONFIG_H
#include "config.h"
#include "Arduino.h"
#include "internal.h"
#include "opusfile.h"

#define OP_PAGE_SIZE_MAX  (65307)

#define OP_CHUNK_SIZE     (65536)
#define OP_CHUNK_SIZE_MAX (1024*(int32_t)1024)
#define OP_READ_SIZE      (2048)

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
/* Many, many internal helpers. The intention is not to be confusing. Rampant duplication and monolithic function
   implementation (though we do have some large, omnibus functions still) would be harder to understand anyway.
   The high level functions are last. Begin grokking near the end of the file if you prefer to read things top-down.*/

/* The read/seek functions track absolute position within the stream.*/
/* Read a little more data from the file/pipe into the ogg_sync framer. _nbytes: The maximum number of bytes to read.
   Return: A positive number of bytes read on success, 0 on end-of-file, or a negative value on failure.*/
static int op_get_data(OggOpusFile *_of, int _nbytes) {
    unsigned char *buffer;
    int nbytes;
    OP_ASSERT(_nbytes>0);
    buffer = (unsigned char*) ogg_sync_buffer(&_of->oy, _nbytes);
    nbytes = (int) (*_of->callbacks.read)(_of->stream, buffer, _nbytes);
//    log_i("nbytes gelesen %i", nbytes);
    OP_ASSERT(nbytes<=_nbytes);
    if(nbytes > 0) ogg_sync_wrote(&_of->oy, nbytes);
    return nbytes;
}
//----------------------------------------------------------------------------------------------------------------------
/*Save a tiny smidge of verbosity to make the code more readable.*/
static int op_seek_helper(OggOpusFile *_of, int64_t _offset) {
    if(_offset == _of->offset) return 0;
    if(_of->callbacks.seek == NULL || (*_of->callbacks.seek)(_of->stream, _offset, SEEK_SET)) {
        return OP_EREAD;
    }
    _of->offset = _offset;
    ogg_sync_reset(&_of->oy);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/* Get the current position indicator of the underlying stream. This should be the same as the value reported
   by tell().*/
static int64_t op_position(const OggOpusFile *_of) {
    /*The current position indicator is _not_ simply offset.
     We may also have unprocessed, buffered data in the sync state.*/
    return _of->offset + _of->oy.fill - _of->oy.returned;
}
//----------------------------------------------------------------------------------------------------------------------
/*From the head of the stream, get the next page. _boundary specifies if the function is allowed to fetch more data
  from the stream (and how much) or only use internally buffered data.
  _boundary: -1: Unbounded search. 0: Read no additional data. Use only cached data.
  n: Search for the start of a new page up to file position n. Return: n>=0: Found a page at absolute offset n.
  OP_FALSE:   Hit the _boundary limit.
  OP_EREAD:   An underlying read operation failed.
  OP_BADLINK: We hit end-of-file before reaching _boundary.*/
static int64_t op_get_next_page(OggOpusFile *_of, ogg_page *_og, int64_t _boundary) {
    while(_boundary <= 0 || _of->offset < _boundary) {
        int more;
        more = ogg_sync_pageseek(&_of->oy, _og);
        /*Skipped (-more) bytes.*/
        if(more < 0)
            _of->offset -= more;
        else if(more == 0) {
            int read_nbytes;
            int ret;
            /*Send more paramedics.*/
            if(!_boundary) return OP_FALSE;
            if(_boundary < 0)
                read_nbytes = OP_READ_SIZE;
            else {
                int64_t position;
                position = op_position(_of);
                if(position >= _boundary) return OP_FALSE;
                read_nbytes = (int) _min(_boundary - position, OP_READ_SIZE);
            }
            ret = op_get_data(_of, read_nbytes);
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
            page_offset = _of->offset;
            _of->offset += more;
            OP_ASSERT(page_offset>=0);
            return page_offset;
        }
    }
    return OP_FALSE;
}
//----------------------------------------------------------------------------------------------------------------------
static int op_add_serialno(const ogg_page *_og, uint32_t **_serialnos, int *_nserialnos, int *_cserialnos) {
    uint32_t *serialnos;
    int nserialnos;
    int cserialnos;
    uint32_t s;
    s = ogg_page_serialno(_og);
    serialnos = *_serialnos;
    nserialnos = *_nserialnos;
    cserialnos = *_cserialnos;
    if(nserialnos >= cserialnos) {
        if(cserialnos > INT_MAX / (int) sizeof(*serialnos) - 1 >> 1) {
            return OP_EFAULT;
        }
        cserialnos = 2 * cserialnos + 1;
        OP_ASSERT(nserialnos<cserialnos);
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
/*Returns nonzero if found.*/
static int op_lookup_serialno(uint32_t _s, const uint32_t *_serialnos, int _nserialnos) {
    int i;
    for(i = 0; i < _nserialnos && _serialnos[i] != _s; i++)
        ;
    return i < _nserialnos;
}
//----------------------------------------------------------------------------------------------------------------------
static int op_lookup_page_serialno(const ogg_page *_og, const uint32_t *_serialnos, int _nserialnos) {
    return op_lookup_serialno(ogg_page_serialno(_og), _serialnos, _nserialnos);
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
/*Find the last page beginning before _offset with a valid granule position.
  There is no '_boundary' parameter as it will always have to read more data.
  This is much dirtier than the above, as Ogg doesn't have any backward search
   linkage.
  This search prefers pages of the specified serial number.
  If a page of the specified serial number is spotted during the
   seek-back-and-read-forward, it will return the info of last page of the
   matching serial number, instead of the very last page, unless the very last
   page belongs to a different link than preferred serial number.
  If no page of the specified serial number is seen, it will return the info of
   the last page.
  [out] _sr:   Returns information about the page that was found on success.
  _offset:     The _offset before which to find a page.
               Any page returned will consist of data entirely before _offset.
  _serialno:   The preferred serial number.
               If a page with this serial number is found, it will be returned
                even if another page in the same link is found closer to
                _offset.
               This is purely opportunistic: there is no guarantee such a page
                will be found if it exists.
  _serialnos:  The list of serial numbers in the link that contains the
                preferred serial number.
  _nserialnos: The number of serial numbers in the current link.
  Return: 0 on success, or a negative value on failure.
          OP_EREAD:    Failed to read more data (error or EOF).
          OP_EBADLINK: We couldn't find a page even after seeking back to the
                        start of the stream.*/
static int op_get_prev_page_serial(OggOpusFile *_of, OpusSeekRecord_t *_sr, int64_t _offset, uint32_t _serialno,
                                   const uint32_t *_serialnos, int _nserialnos) {
    OpusSeekRecord_t preferred_sr;
    ogg_page og;
    int64_t begin;
    int64_t end;
    int64_t original_end;
    int32_t chunk_size;
    int preferred_found;
    original_end = end = begin = _offset;
    preferred_found = 0;
    _offset = -1;
    chunk_size = OP_CHUNK_SIZE;
    do {
        int64_t search_start;
        int ret;
        OP_ASSERT(chunk_size>=OP_PAGE_SIZE_MAX);
        begin = _max(begin - chunk_size, 0);
        ret = op_seek_helper(_of, begin);
        if(ret < 0) return ret;
        search_start = begin;
        while(_of->offset < end) {
            int64_t llret;
            uint32_t serialno;
            llret = op_get_next_page(_of, &og, end);
            if(llret<OP_FALSE)
                return (int) llret;
            else if(llret == OP_FALSE) break;
            serialno = ogg_page_serialno(&og);
            /*Save the information for this page.
             We're not interested in the page itself... just the serial number, byte
             offset, page size, and granule position.*/
            _sr->search_start = search_start;
            _sr->offset = _offset = llret;
            _sr->serialno = serialno;
            OP_ASSERT(_of->offset-_offset>=0); OP_ASSERT(_of->offset-_offset<=OP_PAGE_SIZE_MAX);
            _sr->size = (int32_t) (_of->offset - _offset);
            _sr->gp = ogg_page_granulepos(&og);
            /*If this page is from the stream we're looking for, remember it.*/
            if(serialno == _serialno) {
                preferred_found = 1;
                *&preferred_sr = *_sr;
            }
            if(!op_lookup_serialno(serialno, _serialnos, _nserialnos)) {
                /*We fell off the end of the link, which means we seeked back too far
                 and shouldn't have been looking in that link to begin with.
                 If we found the preferred serial number, forget that we saw it.*/
                preferred_found = 0;
            }
            search_start = llret + 1;
        }
        /*We started from the beginning of the stream and found nothing.
         This should be impossible unless the contents of the stream changed out
         from under us after we read from it.*/
        if((!begin) && (_offset < 0)) return OP_EBADLINK;
        /*Bump up the chunk size.
         This is mildly helpful when seeks are very expensive (http).*/
        chunk_size = _min(2 * chunk_size, OP_CHUNK_SIZE_MAX);
        /*Avoid quadratic complexity if we hit an invalid patch of the file.*/
        end = _min(begin+OP_PAGE_SIZE_MAX-1, original_end);
    } while(_offset < 0);
    if(preferred_found) *_sr = *&preferred_sr;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Find the last page beginning before _offset with the given serial number and
   a valid granule position.
  Unlike the above search, this continues until it finds such a page, but does
   not stray outside the current link.
  We could implement it (inefficiently) by calling op_get_prev_page_serial()
   repeatedly until it returned a page that had both our preferred serial
   number and a valid granule position, but doing it with a separate function
   allows us to avoid repeatedly re-scanning valid pages from other streams as
   we seek-back-and-read-forward.
  [out] _gp:   Returns the granule position of the page that was found on
                success.
  _offset:     The _offset before which to find a page.
               Any page returned will consist of data entirely before _offset.
  _serialno:   The target serial number.
  _serialnos:  The list of serial numbers in the link that contains the
                preferred serial number.
  _nserialnos: The number of serial numbers in the current link.
  Return: The offset of the page on success, or a negative value on failure.
          OP_EREAD:    Failed to read more data (error or EOF).
          OP_EBADLINK: We couldn't find a page even after seeking back past the
                        beginning of the link.*/
static int64_t op_get_last_page(OggOpusFile *_of, int64_t *_gp, int64_t _offset, uint32_t _serialno,
                                const uint32_t *_serialnos, int _nserialnos) {
    ogg_page og;
    int64_t gp;
    int64_t begin;
    int64_t end;
    int64_t original_end;
    int32_t chunk_size;
    /*The target serial number must belong to the current link.*/
    OP_ASSERT(op_lookup_serialno(_serialno,_serialnos,_nserialnos));
    original_end = end = begin = _offset;
    _offset = -1;
    /*We shouldn't have to initialize gp, but gcc is too dumb to figure out that
     ret>=0 implies we entered the if(page_gp!=-1) block at least once.*/
    gp = -1;
    chunk_size = OP_CHUNK_SIZE;
    do {
        int left_link;
        int ret;
        OP_ASSERT(chunk_size>=OP_PAGE_SIZE_MAX);
        begin = _max(begin - chunk_size, 0);
        ret = op_seek_helper(_of, begin);
        if(ret < 0) return ret;
        left_link = 0;
        while(_of->offset < end) {
            int64_t llret;
            uint32_t serialno;
            llret = op_get_next_page(_of, &og, end);
            if(llret<OP_FALSE)
                return llret;
            else if(llret == OP_FALSE) break;
            serialno = ogg_page_serialno(&og);
            if(serialno == _serialno) {
                int64_t page_gp;
                /*The page is from the right stream...*/
                page_gp = ogg_page_granulepos(&og);
                if(page_gp != -1) {
                    /*And has a valid granule position.
                     Let's remember it.*/
                    _offset = llret;
                    gp = page_gp;
                }
            }
            else if(!op_lookup_serialno(serialno, _serialnos, _nserialnos)) {
                /*We fell off the start of the link, which means we don't need to keep
                 seeking any farther back.*/
                left_link = 1;
            }
        }
        /*We started from at or before the beginning of the link and found nothing.
         This should be impossible unless the contents of the stream changed out
         from under us after we read from it.*/
        if(((left_link) || (!begin)) && (_offset < 0)) {
            return OP_EBADLINK;
        }
        /*Bump up the chunk size.
         This is mildly helpful when seeks are very expensive (http).*/
        chunk_size = _min(2 * chunk_size, OP_CHUNK_SIZE_MAX);
        /*Avoid quadratic complexity if we hit an invalid patch of the file.*/
        end = _min(begin+OP_PAGE_SIZE_MAX-1, original_end);
    } while(_offset < 0);
    *_gp = gp;
    return _offset;
}
//----------------------------------------------------------------------------------------------------------------------
/*Uses the local ogg_stream storage in _of.  This is important for non-streaming input sources.*/
static int op_fetch_headers_impl(OggOpusFile *_of, OpusHead_t *_head, OpusTags_t *_tags, uint32_t **_serialnos,
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
        if(_of->ready_state < OP_STREAMSET) {
            /*We don't have an Opus stream in this link yet, so begin prospective
             stream setup.
             We need a stream to get packets.*/
            ogg_stream_reset_serialno(&_of->os, ogg_page_serialno(_og));
            ogg_stream_pagein(&_of->os, _og);
            if(ogg_stream_packetout(&_of->os, &op) > 0) {
                ret = opus_head_parse(_head, op.packet, op.bytes);
                /*Found a valid Opus header.
                 Continue setup.*/
                if(ret >= 0)
                    _of->ready_state = OP_STREAMSET;
                /*If it's just a stream type we don't recognize, ignore it.
                 Everything else is fatal.*/
                else if(ret != OP_ENOTFORMAT) return ret;
            }
            /*TODO: Should a BOS page with no packets be an error?*/
        }
        /*Get the next page.
         No need to clamp the boundary offset against _of->end, as all errors
         become OP_ENOTFORMAT or OP_EBADHEADER.*/
        if(op_get_next_page(_of,_og, OP_ADV_OFFSET(_of->offset,OP_CHUNK_SIZE))<0) {
            return _of->ready_state < OP_STREAMSET ? OP_ENOTFORMAT : OP_EBADHEADER;
        }
    }
    if(_of->ready_state!=OP_STREAMSET) return OP_ENOTFORMAT;
    /*If the first non-header page belonged to our Opus stream, submit it.*/
    if(_of->os.serialno == ogg_page_serialno(_og)) ogg_stream_pagein(&_of->os, _og);
    /*Loop getting packets.*/
    for(;;) {
        switch(ogg_stream_packetout(&_of->os, &op)){
            case 0: {
                /*Loop getting pages.*/
                for(;;) {
                    /*No need to clamp the boundary offset against _of->end, as all
                     errors become OP_EBADHEADER.*/
                    if(op_get_next_page(_of,_og, OP_ADV_OFFSET(_of->offset,OP_CHUNK_SIZE))<0) {
                        return OP_EBADHEADER;
                    }
                    /*If this page belongs to the correct stream, go parse it.*/
                    if(_of->os.serialno == ogg_page_serialno(_og)) {
                        ogg_stream_pagein(&_of->os, _og);
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
                ret = opus_tags_parse(_tags, op.packet, op.bytes);
                if(ret < 0) return ret;
                /*Make sure the page terminated at the end of the comment header.
                 If there is another packet on the page, or part of a packet, then
                 reject the stream.
                 Otherwise seekable sources won't be able to seek back to the start
                 properly.*/
                ret = ogg_stream_packetout(&_of->os, &op);
                if((ret!=0) || (_og->header[_og->header_len - 1] == 255)) {
                    /*If we fail, the caller assumes our tags are uninitialized.*/
                    opus_tags_clear(_tags);
                    return OP_EBADHEADER;
                }
                return 0;
            }
        }
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
static int op_fetch_headers(OggOpusFile *_of, OpusHead_t *_head, OpusTags_t *_tags, uint32_t **_serialnos,
        int *_nserialnos, int *_cserialnos, ogg_page *_og) {
    ogg_page og;
    int ret;
    if(!_og) {
        /*No need to clamp the boundary offset against _of->end, as all errors
         become OP_ENOTFORMAT.*/
        if(op_get_next_page(_of,&og, OP_ADV_OFFSET(_of->offset,OP_CHUNK_SIZE))<0) {
            return OP_ENOTFORMAT;
        }
        _og = &og;
    }
    _of->ready_state = OP_OPENED;
    ret = op_fetch_headers_impl(_of, _head, _tags, _serialnos, _nserialnos, _cserialnos, _og);
    /*Revert back from OP_STREAMSET to OP_OPENED on failure, to prevent
     double-free of the tags in an unseekable stream.*/
    if(ret < 0) _of->ready_state = OP_OPENED;
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
static int op_granpos_add(int64_t *_dst_gp, int64_t _src_gp, int32_t _delta) {
    /*The code below handles this case correctly, but there's no reason we
     should ever be called with these values, so make sure we aren't.*/
    OP_ASSERT(_src_gp!=-1);
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
static int op_granpos_diff(int64_t *_delta, int64_t _gp_a, int64_t _gp_b) {
    int gp_a_negative;
    int gp_b_negative;
    /*The code below handles these cases correctly, but there's no reason we
     should ever be called with these values, so make sure we aren't.*/
    OP_ASSERT(_gp_a!=-1); OP_ASSERT(_gp_b!=-1);
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
static int op_granpos_cmp(int64_t _gp_a, int64_t _gp_b) {
    /*The invalid granule position -1 should behave like NaN: neither greater
     than nor less than any other granule position, nor equal to any other
     granule position, including itself.
     However, that means there isn't anything we could sensibly return from this
     function for it.*/
    OP_ASSERT(_gp_a!=-1); OP_ASSERT(_gp_b!=-1);
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
static int op_get_packet_duration(const unsigned char *_data, int _len) {
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
  _of->op_count is set to the number of packets collected.
  [out] _durations: Returns the durations of the individual packets.
  Return: The total duration of all packets, or OP_HOLE if there was a hole.*/
static int32_t op_collect_audio_packets(OggOpusFile *_of, int _durations[255]) {
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
        ret = ogg_stream_packetout(&_of->os, _of->op + op_count);
        if(!ret) break;
        if(ret < 0) {
            /*We shouldn't get holes in the middle of pages.*/
            OP_ASSERT(op_count==0);
            /*Set the return value and break out of the loop.
             We want to make sure op_count gets set to 0, because we've ingested a
             page, so any previously loaded packets are now invalid.*/
            total_duration = OP_HOLE;
            break;
        }
        /*Unless libogg is broken, we can't get more than 255 packets from a
         single page.*/
        OP_ASSERT(op_count<255);
        _durations[op_count] = op_get_packet_duration(_of->op[op_count].packet, _of->op[op_count].bytes);
        if(_durations[op_count] > 0) {
            /*With at most 255 packets on a page, this can't overflow.*/
            total_duration += _durations[op_count++];
        }
        /*Ignore packets with an invalid TOC sequence.*/
        else if(op_count > 0) {
            /*But save the granule position, if there was one.*/
            _of->op[op_count - 1].granulepos = _of->op[op_count].granulepos;
        }
    }
    _of->op_pos = 0;
    _of->op_count = op_count;
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
static int op_find_initial_pcm_offset(OggOpusFile *_of, OggOpusLink_t *_link, ogg_page *_og) {
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
    serialno = _of->os.serialno;
    op_count = 0;
    /*We shouldn't have to initialize total_duration, but gcc is too dumb to
     figure out that op_count>0 implies we've been through the whole loop at
     least once.*/
    total_duration = 0;
    do {
        page_offset = op_get_next_page(_of, _og, _of->end);
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
            if(_link->head.pre_skip > 0) {
                free(durations);
                return OP_EBADTIMESTAMP;
            }
            _link->pcm_file_offset = 0;
            /*Set pcm_end and end_offset so we can skip the call to
             op_find_final_pcm_offset().*/
            _link->pcm_start = _link->pcm_end = 0;
            _link->end_offset = _link->data_offset;
            free(durations);
            return 0;
        }
        /*Similarly, if we hit the next link in the chain, we've gone too far.*/
        if(ogg_page_bos(_og)) {
            if(_link->head.pre_skip > 0) {
                free(durations);
                return OP_EBADTIMESTAMP;
            }
            /*Set pcm_end and end_offset so we can skip the call to
             op_find_final_pcm_offset().*/
            _link->pcm_file_offset = 0;
            _link->pcm_start = _link->pcm_end = 0;
            _link->end_offset = _link->data_offset;
            /*Tell the caller we've got a buffered page for them.*/
            free(durations);
            return 1;
        }
        /*Ignore pages from other streams (not strictly necessary, because of the
         checks in ogg_stream_pagein(), but saves some work).*/
        if(serialno != (uint32_t) ogg_page_serialno(_og)) continue;
        ogg_stream_pagein(&_of->os, _og);
        /*Bitrate tracking: add the header's bytes here.
         The body bytes are counted when we consume the packets.*/
        _of->bytes_tracked += _og->header_len;
        /*Count the durations of all packets in the page.*/
        do
            total_duration = op_collect_audio_packets(_of, durations);
        /*Ignore holes.*/
        while(total_duration < 0);
        op_count = _of->op_count;
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
    cur_page_gp = _of->op[op_count - 1].granulepos;
    /*But getting a packet without a valid granule position on the page is not
     okay.*/
    if(cur_page_gp == -1) {
        free(durations);
        return OP_EBADTIMESTAMP;
    }
    cur_page_eos = _of->op[op_count - 1].e_o_s;
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
            if(op_granpos_cmp(cur_page_gp, _link->head.pre_skip) < 0) {
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
                _of->op[pi].granulepos = prev_packet_gp = cur_page_gp;
                /*Move the EOS flag to this packet, if necessary, so we'll trim the
                 samples.*/
                _of->op[pi].e_o_s = 1;
                continue;
            }
        }
        /*Update the granule position as normal.*/
        op_granpos_add(&_of->op[pi].granulepos, prev_packet_gp, durations[pi]);
        prev_packet_gp = _of->op[pi].granulepos;
    }
    /*Update the packet count after end-trimming.*/
    _of->op_count = pi;
    _of->cur_discard_count = _link->head.pre_skip;
    _link->pcm_file_offset = 0;
    _of->prev_packet_gp = _link->pcm_start = pcm_start;
    _of->prev_page_offset = page_offset;
    free(durations);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Starting from current cursor position, get the final PCM offset of the
   previous page.
  This also validates the duration of the link, which, while not strictly
   required by the spec, we need to ensure duration calculations don't
   overflow.
  This is only done for seekable sources.
  We must validate that op_find_initial_pcm_offset() succeeded for this link
   before calling this function, otherwise it will scan the entire stream
   backwards until it reaches the start, and then fail.*/
static int op_find_final_pcm_offset(OggOpusFile *_of, const uint32_t *_serialnos, int _nserialnos,
        OggOpusLink_t *_link, int64_t _offset, uint32_t _end_serialno, int64_t _end_gp,
        int64_t *_total_duration) {
    int64_t total_duration;
    int64_t duration;
    uint32_t cur_serialno;
    /*For the time being, fetch end PCM offset the simple way.*/
    cur_serialno = _link->serialno;
    if(_end_serialno != cur_serialno || _end_gp == -1) {
        _offset = op_get_last_page(_of, &_end_gp, _offset, cur_serialno, _serialnos, _nserialnos);
        if(_offset < 0) return (int) _offset;
    }
    /*At worst we should have found the first page with completed packets.*/
    if(_offset < _link->data_offset) return OP_EBADLINK;
    /*This implementation requires that the difference between the first and last
     granule positions in each link be representable in a signed, 64-bit
     number, and that each link also have at least as many samples as the
     pre-skip requires.*/
    if((op_granpos_diff(&duration, _end_gp, _link->pcm_start) < 0) || (duration<_link->head.pre_skip)) {
        return OP_EBADTIMESTAMP;
    }
    /*We also require that the total duration be representable in a signed,
     64-bit number.*/
    duration -= _link->head.pre_skip;
    total_duration = *_total_duration;
    if(INT64_MAX-duration<total_duration) return OP_EBADTIMESTAMP;
    *_total_duration = total_duration + duration;
    _link->pcm_end = _end_gp;
    _link->end_offset = _offset;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Rescale the number _x from the range [0,_from] to [0,_to]. _from and _to must be positive.*/
static int64_t op_rescale64(int64_t _x, int64_t _from, int64_t _to) {
    int64_t frac;
    int64_t ret;
    int i;
    if(_x >= _from) return _to;
    if(_x <= 0) return 0;
    frac = 0;
    for(i = 0; i < 63; i++) {
        frac <<= 1;
        OP_ASSERT(_x<=_from);
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

/*Try to estimate the location of the next link using the current seek
   records, assuming the initial granule position of any streams we've found is
   0.*/
static int64_t op_predict_link_start(const OpusSeekRecord_t *_sr, int _nsr, int64_t _searched, int64_t _end_searched,
        int32_t _bias) {
    int64_t bisect;
    int sri;
    int srj;
    /*Require that we be at least OP_CHUNK_SIZE from the end.
     We don't require that we be at least OP_CHUNK_SIZE from the beginning,
     because if we are we'll just scan forward without seeking.*/
    _end_searched -= OP_CHUNK_SIZE;
    if(_searched >= _end_searched) return -1;
    bisect = _end_searched;
    for(sri = 0; sri < _nsr; sri++) {
        int64_t gp1;
        int64_t gp2_min;
        uint32_t serialno1;
        int64_t offset1;
        /*If the granule position is negative, either it's invalid or we'd cause
         overflow.*/
        gp1 = _sr[sri].gp;
        if(gp1 < 0) continue;
        /*We require some minimum distance between granule positions to make an
         estimate.
         We don't actually know what granule position scheme is being used,
         because we have no idea what kind of stream these came from.
         Therefore we require a minimum spacing between them, with the
         expectation that while bitrates and granule position increments might
         vary locally in quite complex ways, they are globally smooth.*/
        if(op_granpos_add(&gp2_min,gp1,OP_GP_SPACING_MIN)<0) {
            /*No granule position would satisfy us.*/
            continue;
        }
        offset1 = _sr[sri].offset;
        serialno1 = _sr[sri].serialno;
        for(srj = sri; srj-- > 0;) {
            int64_t gp2;
            int64_t offset2;
            int64_t num;
            int64_t den;
            int64_t ipart;
            gp2 = _sr[srj].gp;
            if(gp2 < gp2_min) continue;
            /*Oh, and also make sure these came from the same stream.*/
            if(_sr[srj].serialno != serialno1) continue;
            offset2 = _sr[srj].offset;
            /*For once, we can subtract with impunity.*/
            den = gp2 - gp1;
            ipart = gp2 / den;
            num = offset2 - offset1;
            OP_ASSERT(num>0);
            if(ipart > 0 && (offset2 - _searched) / ipart < num) continue;
            offset2 -= ipart * num;
            gp2 -= ipart * den;
            offset2 -= op_rescale64(gp2, den, num) - _bias;
            if(offset2 < _searched) continue;
            bisect = _min(bisect, offset2);
            break;
        }
    }
    return bisect >= _end_searched ? -1 : bisect;
}
//----------------------------------------------------------------------------------------------------------------------
/*Finds each bitstream link, one at a time, using a bisection search.
  This has to begin by knowing the offset of the first link's initial page.*/
static int op_bisect_forward_serialno(OggOpusFile *_of, int64_t _searched, OpusSeekRecord_t *_sr, int _csr,
        uint32_t **_serialnos, int *_nserialnos, int *_cserialnos) {
    ogg_page og;
    OggOpusLink_t *links;
    int nlinks;
    int clinks;
    uint32_t *serialnos;
    int nserialnos;
    int64_t total_duration;
    int nsr;
    int ret;
    links = _of->links;
    nlinks = clinks = _of->nlinks;
    total_duration = 0;
    /*We start with one seek record, for the last page in the file.
     We build up a list of records for places we seek to during link
     enumeration.
     This list is kept sorted in reverse order.
     We only care about seek locations that were _not_ in the current link,
     therefore we can add them one at a time to the end of the list as we
     improve the lower bound on the location where the next link starts.*/
    nsr = 1;
    for(;;) {
        int64_t end_searched;
        int64_t bisect;
        int64_t next;
        int64_t last;
        int64_t end_offset;
        int64_t end_gp;
        int sri;
        serialnos = *_serialnos;
        nserialnos = *_nserialnos;
        if(nlinks >= clinks) {
            if(clinks>INT_MAX-1>>1) return OP_EFAULT;
            clinks = 2 * clinks + 1;
            OP_ASSERT(nlinks<clinks);
            links = (OggOpusLink_t*) realloc(links, sizeof(*links) * clinks);
            if(links==NULL) return OP_EFAULT;
            _of->links = links;
        }
        /*Invariants:
         We have the headers and serial numbers for the link beginning at 'begin'.
         We have the offset and granule position of the last page in the file
         (potentially not a page we care about).*/
        /*Scan the seek records we already have to save us some bisection.*/
        for(sri = 0; sri < nsr; sri++) {
            if(op_lookup_serialno(_sr[sri].serialno, serialnos, nserialnos)) break;
        }
        /*Is the last page in our current list of serial numbers?*/
        if(sri <= 0) break;
        /*Last page wasn't found.
         We have at least one more link.*/
        last = -1;
        end_searched = _sr[sri - 1].search_start;
        next = _sr[sri - 1].offset;
        end_gp = -1;
        if(sri < nsr) {
            _searched = _sr[sri].offset + _sr[sri].size;
            if(_sr[sri].serialno == links[nlinks - 1].serialno) {
                end_gp = _sr[sri].gp;
                end_offset = _sr[sri].offset;
            }
        }
        nsr = sri;
        bisect = -1;
        /*If we've already found the end of at least one link, try to pick the
         first bisection point at twice the average link size.
         This is a good choice for files with lots of links that are all about the
         same size.*/
        if(nlinks > 1) {
            int64_t last_offset;
            int64_t avg_link_size;
            int64_t upper_limit;
            last_offset = links[nlinks - 1].offset;
            avg_link_size = last_offset / (nlinks - 1);
            upper_limit = end_searched - OP_CHUNK_SIZE - avg_link_size;
            if((last_offset>_searched-avg_link_size) && (last_offset < upper_limit)) {
                bisect = last_offset + avg_link_size;
                if(bisect < upper_limit) bisect += avg_link_size;
            }
        }
        /*We guard against garbage separating the last and first pages of two
         links below.*/
        while(_searched < end_searched) {
            int32_t next_bias;
            /*If we don't have a better estimate, use simple bisection.*/
            if(bisect == -1) bisect = _searched + (end_searched - _searched >> 1);
            /*If we're within OP_CHUNK_SIZE of the start, scan forward.*/
            if(bisect - _searched < OP_CHUNK_SIZE)
                bisect = _searched;
            /*Otherwise we're skipping data.
             Forget the end page, if we saw one, as we might miss a later one.*/
            else
                end_gp = -1;
            ret = op_seek_helper(_of, bisect);
            if(ret < 0) return ret;
            last = op_get_next_page(_of, &og, _sr[nsr - 1].offset);
            if(last<OP_FALSE) return (int) last;
            next_bias = 0;
            if(last == OP_FALSE)
                end_searched = bisect;
            else {
                uint32_t serialno;
                int64_t gp;
                serialno = ogg_page_serialno(&og);
                gp = ogg_page_granulepos(&og);
                if(!op_lookup_serialno(serialno, serialnos, nserialnos)) {
                    end_searched = bisect;
                    next = last;
                    /*In reality we should always have enough room, but be paranoid.*/
                    if(nsr < _csr) {
                        _sr[nsr].search_start = bisect;
                        _sr[nsr].offset = last;
                        OP_ASSERT(_of->offset-last>=0); OP_ASSERT(_of->offset-last<=OP_PAGE_SIZE_MAX);
                        _sr[nsr].size = (int32_t) (_of->offset - last);
                        _sr[nsr].serialno = serialno;
                        _sr[nsr].gp = gp;
                        nsr++;
                    }
                }
                else {
                    _searched = _of->offset;
                    next_bias = OP_CHUNK_SIZE;
                    if(serialno == links[nlinks - 1].serialno) {
                        /*This page was from the stream we want, remember it.
                         If it's the last such page in the link, we won't have to go back
                         looking for it later.*/
                        end_gp = gp;
                        end_offset = last;
                    }
                }
            }
            bisect = op_predict_link_start(_sr, nsr, _searched, end_searched, next_bias);
        }
        /*Bisection point found.
         Get the final granule position of the previous link, assuming
         op_find_initial_pcm_offset() didn't already determine the link was
         empty.*/
        if(links[nlinks - 1].pcm_end == -1) {
            if(end_gp == -1) {
                /*If we don't know where the end page is, we'll have to seek back and
                 look for it, starting from the end of the link.*/
                end_offset = next;
                /*Also forget the last page we read.
                 It won't be available after the seek.*/
                last = -1;
            }
            ret = op_find_final_pcm_offset(_of, serialnos, nserialnos, links + nlinks - 1, end_offset,
                    links[nlinks - 1].serialno, end_gp, &total_duration);
            if(ret < 0) return ret;
        }
        if(last != next) {
            /*The last page we read was not the first page the next link.
             Move the cursor position to the offset of that first page.
             This only performs an actual seek if the first page of the next link
             does not start at the end of the last page from the current Opus
             stream with a valid granule position.*/
            ret = op_seek_helper(_of, next);
            if(ret < 0) return ret;
        }
        ret = op_fetch_headers(_of, &links[nlinks].head, &links[nlinks].tags, _serialnos, _nserialnos, _cserialnos,
                last != next ? NULL : &og);
        if(ret < 0) return ret;
        links[nlinks].offset = next;
        links[nlinks].data_offset = _of->offset;
        links[nlinks].serialno = _of->os.serialno;
        links[nlinks].pcm_end = -1;
        /*This might consume a page from the next link, however the next bisection
         always starts with a seek.*/
        ret = op_find_initial_pcm_offset(_of, links + nlinks, NULL);
        if(ret < 0) return ret;
        links[nlinks].pcm_file_offset = total_duration;
        _searched = _of->offset;
        /*Mark the current link count so it can be cleaned up on error.*/
        _of->nlinks = ++nlinks;
    }
    /*Last page is in the starting serialno list, so we've reached the last link.
     Now find the last granule position for it (if we didn't the first time we
     looked at the end of the stream, and if op_find_initial_pcm_offset()
     didn't already determine the link was empty).*/
    if(links[nlinks - 1].pcm_end == -1) {
        ret = op_find_final_pcm_offset(_of, serialnos, nserialnos, links + nlinks - 1, _sr[0].offset, _sr[0].serialno,
                _sr[0].gp, &total_duration);
        if(ret < 0) return ret;
    }
    /*Trim back the links array if necessary.*/
    links = (OggOpusLink_t*) realloc(links, sizeof(*links) * nlinks);
    if(links!=NULL) _of->links = links;
    /*We also don't need these anymore.*/
    free(*_serialnos);
    *_serialnos = NULL;
    *_cserialnos = *_nserialnos = 0;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
static void op_update_gain(OggOpusFile *_of) {
    OpusHead_t *head;
    int32_t gain_q8;
    int li;
    /*If decode isn't ready, then we'll apply the gain when we initialize the
     decoder.*/
    if(_of->ready_state < OP_INITSET) return;
    gain_q8 = _of->gain_offset_q8;
    li = _of->seekable ? _of->cur_link : 0;
    head = &_of->links[li].head;
    /*We don't have to worry about overflow here because the header gain and
     track gain must lie in the range [-32768,32767], and the user-supplied
     offset has been pre-clamped to [-98302,98303].*/
    switch(_of->gain_type){
        case OP_ALBUM_GAIN: {
            int album_gain_q8;
            album_gain_q8 = 0;
            opus_tags_get_album_gain(&_of->links[li].tags, &album_gain_q8);
            gain_q8 += album_gain_q8;
            gain_q8 += head->output_gain;
        }
            break;
        case OP_TRACK_GAIN: {
            int track_gain_q8;
            track_gain_q8 = 0;
            opus_tags_get_track_gain(&_of->links[li].tags, &track_gain_q8);
            gain_q8 += track_gain_q8;
            gain_q8 += head->output_gain;
        }
            break;
        case OP_HEADER_GAIN:
            gain_q8 += head->output_gain;
            break;
        case OP_ABSOLUTE_GAIN:
            break;
        default:
            OP_ASSERT(0);
    }
    gain_q8 = OP_CLAMP(-32768, gain_q8, 32767);
    OP_ASSERT(_of->od!=NULL);
    opus_multistream_decoder_ctl(_of->od, OPUS_SET_GAIN(gain_q8));

}
//----------------------------------------------------------------------------------------------------------------------
static int op_make_decode_ready(OggOpusFile *_of) {
    const OpusHead_t *head;
    int li;
    int stream_count;
    int coupled_count;
    int channel_count;
    if(_of->ready_state > OP_STREAMSET) return 0;
    if(_of->ready_state < OP_STREAMSET) return OP_EFAULT;
    li = _of->seekable ? _of->cur_link : 0;
    head = &_of->links[li].head;
    stream_count = head->stream_count;
    coupled_count = head->coupled_count;
    channel_count = head->channel_count;
    /*Check to see if the current decoder is compatible with the current link.*/
    if(_of->od != NULL && _of->od_stream_count == stream_count && _of->od_coupled_count == coupled_count
            && _of->od_channel_count == channel_count
            && memcmp(_of->od_mapping, head->mapping, sizeof(*head->mapping) * channel_count) == 0) {
        opus_multistream_decoder_ctl(_of->od, OPUS_RESET_STATE);
    }
    else {
        int err;
        opus_multistream_decoder_destroy(_of->od);
        _of->od = opus_multistream_decoder_create(48000, channel_count, stream_count, coupled_count, head->mapping,
                &err);
        if(_of->od == NULL) return OP_EFAULT;
        _of->od_stream_count = stream_count;
        _of->od_coupled_count = coupled_count;
        _of->od_channel_count = channel_count;
        memcpy(_of->od_mapping, head->mapping, sizeof(*head->mapping) * channel_count);
    }
    _of->ready_state = OP_INITSET;
    _of->bytes_tracked = 0;
    _of->samples_tracked = 0;
    op_update_gain(_of);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
static int op_open_seekable2_impl(OggOpusFile *_of) {
    /*64 seek records should be enough for anybody.
     Actually, with a bisection search in a 63-bit range down to OP_CHUNK_SIZE
     granularity, much more than enough.*/
    OpusSeekRecord_t *sr = (OpusSeekRecord_t*) malloc(64 * sizeof(OpusSeekRecord_t));
    int64_t data_offset;
    int ret;
    /*We can seek, so set out learning all about this file.*/
    (*_of->callbacks.seek)(_of->stream, 0, SEEK_END);
    _of->offset = _of->end = (*_of->callbacks.tell)(_of->stream);
    if(_of->end < 0) {
        free(sr);
        return OP_EREAD;
    }
    data_offset = _of->links[0].data_offset;
    if(_of->end < data_offset) {
        free(sr);
        return OP_EBADLINK;
    }
    /*Get the offset of the last page of the physical bitstream, or, if we're
     lucky, the last Opus page of the first link, as most Ogg Opus files will
     contain a single logical bitstream.*/
    ret = op_get_prev_page_serial(_of, sr, _of->end, _of->links[0].serialno, _of->serialnos, _of->nserialnos);
    if(ret < 0) {
        free(sr);
        return ret;
    }
    /*If there's any trailing junk, forget about it.*/
    _of->end = sr[0].offset + sr[0].size;
    if(_of->end < data_offset) {
        free(sr);
        return OP_EBADLINK;
    }
    /*Now enumerate the bitstream structure.*/
    ret = op_bisect_forward_serialno(_of, data_offset, sr, sizeof(sr) / sizeof(*sr), &_of->serialnos, &_of->nserialnos,
            &_of->cserialnos);
    free(sr);
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
static int op_open_seekable2(OggOpusFile *_of) {
    ogg_sync_state oy_start;
    ogg_stream_state *os_start = (ogg_stream_state*) malloc(sizeof(ogg_stream_state));
    ogg_packet *op_start;
    int64_t prev_page_offset;
    int64_t start_offset;
    int start_op_count;
    int ret;
    /*We're partially open and have a first link header state in storage in _of.
     Save off that stream state so we can come back to it.
     It would be simpler to just dump all this state and seek back to
     links[0].data_offset when we're done.
     But we do the extra work to allow us to seek back to _exactly_ the same
     stream position we're at now.
     This allows, e.g., the HTTP backend to continue reading from the original
     connection (if it's still available), instead of opening a new one.
     This means we can open and start playing a normal Opus file with a single
     link and reasonable packet sizes using only two HTTP requests.*/
    start_op_count = _of->op_count;
    /*This is a bit too large to put on the stack unconditionally.*/
    op_start = (ogg_packet*) malloc(sizeof(*op_start) * start_op_count);
    if(op_start == NULL) {
        free(os_start);
        return OP_EFAULT;
    }
    *&oy_start = _of->oy;
    *os_start = _of->os;
    prev_page_offset = _of->prev_page_offset;
    start_offset = _of->offset;
    memcpy(op_start, _of->op, sizeof(*op_start) * start_op_count);
    OP_ASSERT((*_of->callbacks.tell)(_of->stream)==op_position(_of));
    ogg_sync_init(&_of->oy);
    ogg_stream_init(&_of->os, -1);
    ret = op_open_seekable2_impl(_of);
    /*Restore the old stream state.*/
    ogg_stream_clear(&_of->os);
    ogg_sync_clear(&_of->oy);
    *&_of->oy = *&oy_start;
    *&_of->os = *os_start;
    _of->offset = start_offset;
    _of->op_count = start_op_count;
    memcpy(_of->op, op_start, sizeof(*_of->op) * start_op_count);
    free(op_start);
    _of->prev_packet_gp = _of->links[0].pcm_start;
    _of->prev_page_offset = prev_page_offset;
    _of->cur_discard_count = _of->links[0].head.pre_skip;
    if(ret < 0) {
        free(os_start);
        return ret;
    }
    /*And restore the position indicator.*/
    ret = (*_of->callbacks.seek)(_of->stream, op_position(_of), SEEK_SET);
    free(os_start);
    return (ret<0) ? OP_EREAD : 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Clear out the current logical bitstream decoder.*/
static void op_decode_clear(OggOpusFile *_of) {
    /*We don't actually free the decoder.
     We might be able to re-use it for the next link.*/
    _of->op_count = 0;
    _of->od_buffer_size = 0;
    _of->prev_packet_gp = -1;
    _of->prev_page_offset = -1;
    if(!_of->seekable) {
        OP_ASSERT(_of->ready_state>=OP_INITSET);
        opus_tags_clear(&_of->links[0].tags);
    }
    _of->ready_state = OP_OPENED;
}
//----------------------------------------------------------------------------------------------------------------------
static void op_clear(OggOpusFile *_of) {
    OggOpusLink_t *links;
    free(_of->od_buffer);
    if(_of->od != NULL) opus_multistream_decoder_destroy(_of->od);
    links = _of->links;
    if(!_of->seekable) {
        if(_of->ready_state > OP_OPENED || _of->ready_state == OP_PARTOPEN) {
            opus_tags_clear(&links[0].tags);
        }
    }
    else if(links!=NULL) {
        int nlinks;
        int link;
        nlinks = _of->nlinks;
        for(link = 0; link < nlinks; link++)
            opus_tags_clear(&links[link].tags);
    }
    free(links);
    free(_of->serialnos);
    ogg_stream_clear(&_of->os);
    ogg_sync_clear(&_of->oy);
    if(_of->callbacks.close != NULL) (*_of->callbacks.close)(_of->stream);
}
//----------------------------------------------------------------------------------------------------------------------
static int op_open1(OggOpusFile *_of, void *_stream, const OpusFileCallbacks_t *_cb, const unsigned char *_initial_data,
        size_t _initial_bytes) {
    ogg_page og;
    ogg_page *pog;
    int seekable;
    int ret;
    memset(_of, 0, sizeof(*_of));
    if(_initial_bytes>(size_t)LONG_MAX) return OP_EFAULT;
    _of->end = -1;
    _of->stream = _stream;
    *&_of->callbacks = *_cb;
    /*At a minimum, we need to be able to read data.*/
    if(_of->callbacks.read==NULL) return OP_EREAD;
    /*Initialize the framing state.*/
    ogg_sync_init(&_of->oy);
    /*Perhaps some data was previously read into a buffer for testing against
     other stream types.
     Allow initialization from this previously read data (especially as we may
     be reading from a non-seekable stream).
     This requires copying it into a buffer allocated by ogg_sync_buffer() and
     doesn't support seeking, so this is not a good mechanism to use for
     decoding entire files from RAM.*/
    if(_initial_bytes > 0) {
        char *buffer;
        buffer = ogg_sync_buffer(&_of->oy, (long) _initial_bytes);
        memcpy(buffer, _initial_data, _initial_bytes * sizeof(*buffer));
        ogg_sync_wrote(&_of->oy, (long) _initial_bytes);
    }
    /*Can we seek?
     Stevens suggests the seek test is portable.
     It's actually not for files on win32, but we address that by fixing it in
     our callback implementation (see stream.c).*/
    seekable = _cb->seek != NULL && (*_cb->seek)(_stream, 0, SEEK_CUR) != -1;
    /*If seek is implemented, tell must also be implemented.*/
    if(seekable) {
        int64_t pos;
        if(_of->callbacks.tell==NULL) return OP_EINVAL;
        pos = (*_of->callbacks.tell)(_of->stream);
        /*If the current position is not equal to the initial bytes consumed,
         absolute seeking will not work.*/
        if(pos != (int64_t )_initial_bytes) return OP_EINVAL;
    }
    _of->seekable = seekable;
    /*Don't seek yet.
     Set up a 'single' (current) logical bitstream entry for partial open.*/
    _of->links = (OggOpusLink_t*) malloc(sizeof(*_of->links));
    /*The serialno gets filled in later by op_fetch_headers().*/
    ogg_stream_init(&_of->os, -1);
    pog = NULL;
    for(;;) {
        /*Fetch all BOS pages, store the Opus header and all seen serial numbers,
         and load subsequent Opus setup headers.*/
        ret = op_fetch_headers(_of, &_of->links[0].head, &_of->links[0].tags, &_of->serialnos, &_of->nserialnos,
                &_of->cserialnos, pog);
        if(ret < 0) break;
        _of->nlinks = 1;
        _of->links[0].offset = 0;
        _of->links[0].data_offset = _of->offset;
        _of->links[0].pcm_end = -1;
        _of->links[0].serialno = _of->os.serialno;
        /*Fetch the initial PCM offset.*/
        ret = op_find_initial_pcm_offset(_of, _of->links, &og);
        if(seekable || (ret <= 0)) break;
        /*This link was empty, but we already have the BOS page for the next one in
         og.
         We can't seek, so start processing the next link right now.*/
        opus_tags_clear(&_of->links[0].tags);
        _of->nlinks = 0;
        if(!seekable) _of->cur_link++;
        pog = &og;
    }
    if(ret >= 0) _of->ready_state = OP_PARTOPEN;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
static int op_open2(OggOpusFile *_of) {
    int ret;
    OP_ASSERT(_of->ready_state==OP_PARTOPEN);
    if(_of->seekable) {
        _of->ready_state = OP_OPENED;
        ret = op_open_seekable2(_of);
    }
    else
        ret = 0;
    if(ret >= 0) {
        /*We have buffered packets from op_find_initial_pcm_offset().
         Move to OP_INITSET so we can use them.*/
        _of->ready_state = OP_STREAMSET;
        ret = op_make_decode_ready(_of);
        if(ret >= 0) return 0;
    }
    /*Don't auto-close the stream on failure.*/
    _of->callbacks.close = NULL;
    op_clear(_of);
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
OggOpusFile* op_test_callbacks(void *_stream, const OpusFileCallbacks_t *_cb, const unsigned char *_initial_data,
        size_t _initial_bytes, int *_error) {
    OggOpusFile *of;
    int ret;
    of = (OggOpusFile*) malloc(sizeof(*of));
    ret = OP_EFAULT;
    if(of!=NULL) {
        ret = op_open1(of, _stream, _cb, _initial_data, _initial_bytes);
        if(ret >= 0) {
            if(_error != NULL) *_error = 0;
            return of;
        }
        /*Don't auto-close the stream on failure.*/
        of->callbacks.close = NULL;
        op_clear(of);
        free(of);
    }
    if(_error != NULL) *_error = ret;
    return NULL;
}
//----------------------------------------------------------------------------------------------------------------------
OggOpusFile* op_open_callbacks(void *_stream, const OpusFileCallbacks_t *_cb, const unsigned char *_initial_data,
        size_t _initial_bytes, int *_error) {
    OggOpusFile *of;
    of = op_test_callbacks(_stream, _cb, _initial_data, _initial_bytes, _error);
    if(of!=NULL) {
        int ret;
        ret = op_open2(of);
        if(ret >= 0) return of;
        if(_error != NULL) *_error = ret;
        free(of);
    }
    return NULL;
}
//----------------------------------------------------------------------------------------------------------------------
/*Convenience routine to clean up from failure for the open functions that
 create their own streams.*/
static OggOpusFile* op_test_close_on_failure(void *_stream, const OpusFileCallbacks_t *_cb, int *_error) {
    OggOpusFile *of;
    if(_stream==NULL) {
        if(_error != NULL) *_error = OP_EFAULT;
        return NULL;
    }
    of = op_test_callbacks(_stream, _cb, NULL, 0, _error);
    if(of==NULL) (*_cb->close)(_stream);
    return of;
}
//----------------------------------------------------------------------------------------------------------------------
OggOpusFile* op_test_file(const char *_path, int *_error) {
    OpusFileCallbacks_t cb;
    return op_test_close_on_failure(op_fopen(&cb, _path, "rb"), &cb, _error);
}
//----------------------------------------------------------------------------------------------------------------------
OggOpusFile* op_test_memory(const unsigned char *_data, size_t _size, int *_error) {
    OpusFileCallbacks_t cb;
    return op_test_close_on_failure(op_mem_stream_create(&cb, _data, _size), &cb, _error);
}
//----------------------------------------------------------------------------------------------------------------------
int op_test_open(OggOpusFile *_of) {
    int ret;
    if(_of->ready_state!=OP_PARTOPEN) return OP_EINVAL;
    ret = op_open2(_of);
    /*op_open2() will clear this structure on failure.
     Reset its contents to prevent double-frees in op_free().*/
    if(ret < 0) memset(_of, 0, sizeof(*_of));
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
void op_free(OggOpusFile *_of) {
    if(_of!=NULL) {
        op_clear(_of);
        free(_of);
    }
}
//----------------------------------------------------------------------------------------------------------------------
int op_seekable(const OggOpusFile *_of) {
    return _of->seekable;
}
//----------------------------------------------------------------------------------------------------------------------
int op_link_count(const OggOpusFile *_of) {
    return _of->nlinks;
}
//----------------------------------------------------------------------------------------------------------------------
uint32_t op_serialno(const OggOpusFile *_of, int _li) {
    if(_li >= _of->nlinks) _li = _of->nlinks - 1;
    if(!_of->seekable) _li = 0;
    return _of->links[_li < 0 ? _of->cur_link : _li].serialno;
}
//----------------------------------------------------------------------------------------------------------------------
int op_channel_count(const OggOpusFile *_of, int _li) {
    return op_head(_of, _li)->channel_count;
}
//----------------------------------------------------------------------------------------------------------------------
int64_t op_raw_total(const OggOpusFile *_of, int _li) {
    if((_of->ready_state<OP_OPENED) || (!_of->seekable) || (_li >= _of->nlinks)) {
        return OP_EINVAL;
    }
    if(_li < 0) return _of->end;
    return (_li + 1 >= _of->nlinks ? _of->end : _of->links[_li + 1].offset) - (_li > 0 ? _of->links[_li].offset : 0);
}
//----------------------------------------------------------------------------------------------------------------------
int64_t op_pcm_total(const OggOpusFile *_of, int _li) {
    OggOpusLink_t *links;
    int64_t pcm_total;
    int64_t diff;
    int nlinks;
    nlinks = _of->nlinks;
    if((_of->ready_state<OP_OPENED) || (!_of->seekable) || (_li >= nlinks)) {
        return OP_EINVAL;
    }
    links = _of->links;
    /*We verify that the granule position differences are larger than the
     pre-skip and that the total duration does not overflow during link
     enumeration, so we don't have to check here.*/
    pcm_total = 0;
    if(_li < 0) {
        pcm_total = links[nlinks - 1].pcm_file_offset;
        _li = nlinks - 1;
    }
    op_granpos_diff(&diff, links[_li].pcm_end, links[_li].pcm_start);
    return pcm_total + diff - links[_li].head.pre_skip;
}
//----------------------------------------------------------------------------------------------------------------------
const OpusHead_t* op_head(const OggOpusFile *_of, int _li) {
    if(_li >= _of->nlinks) _li = _of->nlinks - 1;
    if(!_of->seekable) _li = 0;
    return &_of->links[_li < 0 ? _of->cur_link : _li].head;
}
//----------------------------------------------------------------------------------------------------------------------
const OpusTags_t* op_tags(const OggOpusFile *_of, int _li) {
    if(_li >= _of->nlinks) _li = _of->nlinks - 1;
    if(!_of->seekable) {
        if(_of->ready_state < OP_STREAMSET && _of->ready_state != OP_PARTOPEN) {
            return NULL;
        }
        _li = 0;
    }
    else if(_li < 0) _li = _of->ready_state >= OP_STREAMSET ? _of->cur_link : 0;
    return &_of->links[_li].tags;
}
//----------------------------------------------------------------------------------------------------------------------
int op_current_link(const OggOpusFile *_of) {
    if(_of->ready_state<OP_OPENED) return OP_EINVAL;
    return _of->cur_link;
}
//----------------------------------------------------------------------------------------------------------------------
/*Compute an average bitrate given a byte and sample count.
 Return: The bitrate in bits per second.*/
static int32_t op_calc_bitrate(int64_t _bytes, int64_t _samples) {
    if(_samples <= 0) return INT32_MAX;
    /*These rates are absurd, but let's handle them anyway.*/
    if((_bytes>(INT64_MAX-(_samples>>1))/(48000*8))) {
        int64_t den;
        if((_bytes/(INT32_MAX/(48000*8))>=_samples)) {
            return INT32_MAX;
        }
        den = _samples / (48000 * 8);
        return (int32_t) ((_bytes + (den >> 1)) / den);
    }
    /*This can't actually overflow in normal operation: even with a pre-skip of
     545 2.5 ms frames with 8 streams running at 1282*8+1 bytes per packet
     (1275 byte frames + Opus framing overhead + Ogg lacing values), that all
     produce a single sample of decoded output, we still don't top 45 Mbps.
     The only way to get bitrates larger than that is with excessive Opus
     padding, more encoded streams than output channels, or lots and lots of
     Ogg pages with no packets on them.*/
    return (int32_t) _min((_bytes * 48000 * 8 + (_samples >> 1)) / _samples, INT32_MAX);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t op_bitrate(const OggOpusFile *_of, int _li) {
    if((_of->ready_state<OP_OPENED) || (!_of->seekable) || (_li >= _of->nlinks)) {
        return OP_EINVAL;
    }
    return op_calc_bitrate(op_raw_total(_of, _li), op_pcm_total(_of, _li));
}
//----------------------------------------------------------------------------------------------------------------------
int32_t op_bitrate_instant(OggOpusFile *_of) {
    int64_t samples_tracked;
    int32_t ret;
    if((_of->ready_state<OP_OPENED)) return OP_EINVAL;
    samples_tracked = _of->samples_tracked;
    if((samples_tracked == 0)) return OP_FALSE;
    ret = op_calc_bitrate(_of->bytes_tracked, samples_tracked);
    _of->bytes_tracked = 0;
    _of->samples_tracked = 0;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
/*Given a serialno, find a link with a corresponding Opus stream, if it exists.
 Return: The index of the link to which the page belongs, or a negative number
 if it was not a desired Opus bitstream section.*/
static int op_get_link_from_serialno(const OggOpusFile *_of, int _cur_link, int64_t _page_offset,
        uint32_t _serialno) {
    const OggOpusLink_t *links;
    int nlinks;
    int li_lo;
    int li_hi;
    OP_ASSERT(_of->seekable);
    links = _of->links;
    nlinks = _of->nlinks;
    li_lo = 0;
    /*Start off by guessing we're just a multiplexed page in the current link.*/
    li_hi = _cur_link + 1 < nlinks && _page_offset < links[_cur_link + 1].offset ? _cur_link + 1 : nlinks;
    do {
        if(_page_offset >= links[_cur_link].offset)
            li_lo = _cur_link;
        else
            li_hi = _cur_link;
        _cur_link = li_lo + (li_hi - li_lo >> 1);
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
static int op_fetch_and_process_page(OggOpusFile *_of, ogg_page *_og, int64_t _page_offset, int _spanp,
        int _ignore_holes) {
    OggOpusLink_t *links;
    uint32_t cur_serialno;
    int seekable;
    int cur_link;
    int ret;

    /*We shouldn't get here if we have unprocessed packets.*/
    OP_ASSERT(_of->ready_state<OP_INITSET||_of->op_pos>=_of->op_count);

    seekable = _of->seekable;
    links = _of->links;
    cur_link = seekable ? _of->cur_link : 0;
    cur_serialno = links[cur_link].serialno;
    /*Handle one page.*/
    for(;;) {
        ogg_page og;
        OP_ASSERT(_of->ready_state>=OP_OPENED);
        /*If we were given a page to use, use it.*/
        if(_og != NULL) {
            *&og = *_og;
            _og = NULL;
        }
        /*Keep reading until we get a page with the correct serialno.*/
        else
            _page_offset = op_get_next_page(_of, &og, _of->end);
        /*EOF: Leave uninitialized.*/
        if(_page_offset < 0) return _page_offset < OP_FALSE ? (int) _page_offset : OP_EOF;
        if((_of->ready_state>=OP_STREAMSET) && cur_serialno != (uint32_t) ogg_page_serialno(&og)) {
            /*Two possibilities:
             1) Another stream is multiplexed into this logical section, or*/
            if(!ogg_page_bos(&og)) continue;
            /* 2) Our decoding just traversed a bitstream boundary.*/
            if(!_spanp) return OP_EOF;
            if(_of->ready_state>=OP_INITSET) op_decode_clear(_of);
        }
        /*Bitrate tracking: add the header's bytes here.
         The body bytes are counted when we consume the packets.*/
        else
            _of->bytes_tracked += og.header_len;
        /*Do we need to load a new machine before submitting the page?
         This is different in the seekable and non-seekable cases.
         In the seekable case, we already have all the header information loaded
         and cached.
         We just initialize the machine with it and continue on our merry way.
         In the non-seekable (streaming) case, we'll only be at a boundary if we
         just left the previous logical bitstream, and we're now nominally at the
         header of the next bitstream.*/
        if(_of->ready_state<OP_STREAMSET) {
            if(seekable) {
                uint32_t serialno;
                serialno = ogg_page_serialno(&og);
                /*Match the serialno to bitstream section.*/
                OP_ASSERT(cur_link>=0&&cur_link<_of->nlinks);
                if(links[cur_link].serialno != serialno) {
                    /*It wasn't a page from the current link.
                     Is it from the next one?*/
                    if((cur_link + 1 < _of->nlinks && links[cur_link + 1].serialno == serialno)) {
                        cur_link++;
                    }
                    else {
                        int new_link;
                        new_link = op_get_link_from_serialno(_of, cur_link, _page_offset, serialno);
                        /*Not a desired Opus bitstream section.
                         Keep trying.*/
                        if(new_link < 0) continue;
                        cur_link = new_link;
                    }
                }
                cur_serialno = serialno;
                _of->cur_link = cur_link;
                ogg_stream_reset_serialno(&_of->os, serialno);
                _of->ready_state = OP_STREAMSET;
                /*If we're at the start of this link, initialize the granule position
                 and pre-skip tracking.*/
                if(_page_offset <= links[cur_link].data_offset) {
                    _of->prev_packet_gp = links[cur_link].pcm_start;
                    _of->prev_page_offset = -1;
                    _of->cur_discard_count = links[cur_link].head.pre_skip;
                    /*Ignore a hole at the start of a new link (this is common for
                     streams joined in the middle) or after seeking.*/
                    _ignore_holes = 1;
                }
            }
            else {
                do {
                    /*We're streaming.
                     Fetch the two header packets, build the info struct.*/
                    ret = op_fetch_headers(_of, &links[0].head, &links[0].tags,
                    NULL, NULL, NULL, &og);
                    if(ret < 0) return ret;
                    /*op_find_initial_pcm_offset() will suppress any initial hole for us,
                     so no need to set _ignore_holes.*/
                    ret = op_find_initial_pcm_offset(_of, links, &og);
                    if(ret < 0) return ret;
                    _of->links[0].serialno = cur_serialno = _of->os.serialno;
                    _of->cur_link++;
                }
                /*If the link was empty, keep going, because we already have the
                 BOS page of the next one in og.*/
                while(ret > 0);
                /*If we didn't get any packets out of op_find_initial_pcm_offset(),
                 keep going (this is possible if end-trimming trimmed them all).*/
                if(_of->op_count <= 0) continue;
                /*Otherwise, we're done.
                 TODO: This resets bytes_tracked, which misses the header bytes
                 already processed by op_find_initial_pcm_offset().*/
                ret = op_make_decode_ready(_of);
                if(ret < 0) return ret;
                return 0;
            }
        }
        /*The buffered page is the data we want, and we're ready for it.
         Add it to the stream state.*/
        if(_of->ready_state==OP_STREAMSET) {
            ret = op_make_decode_ready(_of);
            if(ret < 0) return ret;
        }
        /*Extract all the packets from the current page.*/
        ogg_stream_pagein(&_of->os, &og);
        if(_of->ready_state>=OP_INITSET) {
            int32_t total_duration;
            int *durations = (int*) malloc(255 * sizeof(int));
            int op_count;
            int report_hole;
            report_hole = 0;
            total_duration = op_collect_audio_packets(_of, durations);
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
                    total_duration = op_collect_audio_packets(_of, durations);
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
                    _of->prev_packet_gp = -1;
                }
            }
            op_count = _of->op_count;
            /*If we found at least one audio data packet, compute per-packet granule
             positions for them.*/
            if(op_count > 0) {
                int64_t diff;
                int64_t prev_packet_gp;
                int64_t cur_packet_gp;
                int64_t cur_page_gp;
                int cur_page_eos;
                int pi;
                cur_page_gp = _of->op[op_count - 1].granulepos;
                cur_page_eos = _of->op[op_count - 1].e_o_s;
                prev_packet_gp = _of->prev_packet_gp;
                if(prev_packet_gp == -1) {
                    int32_t cur_discard_count;
                    /*This is the first call after a raw seek.
                     Try to reconstruct prev_packet_gp from scratch.*/
                    OP_ASSERT(seekable);
                    if(cur_page_eos) {
                        /*If the first page we hit after our seek was the EOS page, and
                         we didn't start from data_offset or before, we don't have
                         enough information to do end-trimming.
                         Proceed to the next link, rather than risk playing back some
                         samples that shouldn't have been played.*/
                        _of->op_count = 0;
                        if(report_hole) {
                            free(durations);
                            return OP_HOLE;
                        }
                        continue;
                    }
                    /*By default discard 80 ms of data after a seek, unless we seek
                     into the pre-skip region.*/
                    cur_discard_count = 80 * 48;
                    cur_page_gp = _of->op[op_count - 1].granulepos;
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
                    _of->cur_discard_count = cur_discard_count;
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
                            _of->op[pi].e_o_s = 1;
                        }
                        else {
                            /*Update the granule position as normal.*/
                            op_granpos_add(&cur_packet_gp, cur_packet_gp, durations[pi]);
                        }
                        _of->op[pi].granulepos = cur_packet_gp;
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
                        OP_ASSERT(total_duration>=0);
                        op_granpos_add(&cur_packet_gp, cur_packet_gp, durations[pi]);
                        _of->op[pi].granulepos = cur_packet_gp;
                    }OP_ASSERT(total_duration==0);
                }
                _of->prev_packet_gp = prev_packet_gp;
                _of->prev_page_offset = _page_offset;
                _of->op_count = op_count = pi;
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
int op_raw_seek(OggOpusFile *_of, int64_t _pos) {
    int ret;
    if(_of->ready_state < OP_OPENED) return OP_EINVAL;
    /*Don't dump the decoder state if we can't seek.*/
    if((!_of->seekable)) return OP_ENOSEEK;
    if((_pos < 0) || (_pos > _of->end)) return OP_EINVAL;
    /*Clear out any buffered, decoded data.*/
    op_decode_clear(_of);
    _of->bytes_tracked = 0;
    _of->samples_tracked = 0;
    ret = op_seek_helper(_of, _pos);
    if(ret < 0) return OP_EREAD;
    ret = op_fetch_and_process_page(_of, NULL, -1, 1, 1);
    /*If we hit EOF, op_fetch_and_process_page() leaves us uninitialized.
     Instead, jump to the end.*/
    if(ret == OP_EOF) {
        int cur_link;
        op_decode_clear(_of);
        cur_link = _of->nlinks - 1;
        _of->cur_link = cur_link;
        _of->prev_packet_gp = _of->links[cur_link].pcm_end;
        _of->cur_discard_count = 0;
        ret = 0;
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
/*Convert a PCM offset relative to the start of the whole stream to a granule
 position in an individual link.*/
static int64_t op_get_granulepos(const OggOpusFile *_of, int64_t _pcm_offset, int *_li) {
    const OggOpusLink_t *links;
    int64_t duration;
    int64_t pcm_start;
    int32_t pre_skip;
    int nlinks;
    int li_lo;
    int li_hi;
    OP_ASSERT(_pcm_offset>=0);
    nlinks = _of->nlinks;
    links = _of->links;
    li_lo = 0;
    li_hi = nlinks;
    do {
        int li;
        li = li_lo + (li_hi - li_lo >> 1);
        if(links[li].pcm_file_offset <= _pcm_offset)
            li_lo = li;
        else
            li_hi = li;
    } while(li_hi - li_lo > 1);
    _pcm_offset -= links[li_lo].pcm_file_offset;
    pcm_start = links[li_lo].pcm_start;
    pre_skip = links[li_lo].head.pre_skip;
    op_granpos_diff(&duration, links[li_lo].pcm_end, pcm_start);
    duration -= pre_skip;
    if(_pcm_offset >= duration) return -1;
    _pcm_offset += pre_skip;
    if(pcm_start > INT64_MAX -_pcm_offset) {
        /*Adding this amount to the granule position would overflow the positive
         half of its 64-bit range.
         Since signed overflow is undefined in C, do it in a way the compiler
         isn't allowed to screw up.*/
        _pcm_offset -= INT64_MAX - pcm_start + 1;
        pcm_start = INT64_MIN;
    }
    pcm_start += _pcm_offset;
    *_li = li_lo;
    return pcm_start;
}
//----------------------------------------------------------------------------------------------------------------------
/*A small helper to determine if an Ogg page contains data that continues onto
 a subsequent page.*/
static int op_page_continues(const ogg_page *_og) {
    int nlacing;
    OP_ASSERT(_og->header_len>=27);
    nlacing = _og->header[26];
    OP_ASSERT(_og->header_len>=27+nlacing);
    /*This also correctly handles the (unlikely) case of nlacing==0, because
     0!=255.*/
    return _og->header[27 + nlacing - 1] == 255;
}
//----------------------------------------------------------------------------------------------------------------------
/*A small helper to buffer the continued packet data from a page.*/
static void op_buffer_continued_data(OggOpusFile *_of, ogg_page *_og) {
    ogg_packet op;
    ogg_stream_pagein(&_of->os, _og);
    /*Drain any packets that did end on this page (and ignore holes).
     We only care about the continued packet data.*/
    while(ogg_stream_packetout(&_of->os, &op))
        ;
}
//----------------------------------------------------------------------------------------------------------------------
/*This controls how close the target has to be to use the current stream
 position to subdivide the initial range.
 Two minutes seems to be a good default.*/
#define OP_CUR_TIME_THRESH (120*48*(int32_t)1000)

//----------------------------------------------------------------------------------------------------------------------
/*Search within link _li for the page with the highest granule position
 preceding (or equal to) _target_gp.
 There is a danger here: missing pages or incorrect frame number information
 in the bitstream could make our task impossible.
 Account for that (and report it as an error condition).*/
static int op_pcm_seek_page(OggOpusFile *_of, int64_t _target_gp, int _li) {
    const OggOpusLink_t *link;
    ogg_page og;
    int64_t pcm_pre_skip;
    int64_t pcm_start;
    int64_t pcm_end;
    int64_t best_gp;
    int64_t diff;
    uint32_t serialno;
    int32_t pre_skip;
    int64_t begin;
    int64_t end;
    int64_t boundary;
    int64_t best;
    int64_t best_start;
    int64_t page_offset;
    int64_t d0;
    int64_t d1;
    int64_t d2;
    int force_bisect;
    int buffering;
    int ret;
    _of->bytes_tracked = 0;
    _of->samples_tracked = 0;
    link = _of->links + _li;
    best_gp = pcm_start = link->pcm_start;
    pcm_end = link->pcm_end;
    serialno = link->serialno;
    best = best_start = begin = link->data_offset;
    page_offset = -1;
    buffering = 0;
    /*We discard the first 80 ms of data after a seek, so seek back that much
     farther.
     If we can't, simply seek to the beginning of the link.*/
    if((op_granpos_add(&_target_gp, _target_gp, -80 * 48) < 0) ||(op_granpos_cmp(_target_gp,pcm_start)<0)) {
        _target_gp = pcm_start;
    }
    /*Special case seeking to the start of the link.*/
    pre_skip = link->head.pre_skip;
    op_granpos_add(&pcm_pre_skip, pcm_start, pre_skip);
    if(op_granpos_cmp(_target_gp, pcm_pre_skip) < 0)
        end = boundary = begin;
    else {
        end = boundary = link->end_offset;

        /*If we were decoding from this link, we can narrow the range a bit.*/
        if(_li == _of->cur_link && _of->ready_state >= OP_INITSET) {
            int64_t offset;
            int op_count;
            op_count = _of->op_count;
            /*The only way the offset can be invalid _and_ we can fail the granule
             position checks below is if someone changed the contents of the last
             page since we read it.
             We'd be within our rights to just return OP_EBADLINK in that case, but
             we'll simply ignore the current position instead.*/
            offset = _of->offset;
            if(op_count > 0 && (offset <= end)) {
                int64_t gp;
                /*Make sure the timestamp is valid.
                 The granule position might be -1 if we collected the packets from a
                 page without a granule position after reporting a hole.*/
                gp = _of->op[op_count - 1].granulepos;
                if((gp != -1)&&(op_granpos_cmp(pcm_start,gp)<0)
                &&(op_granpos_cmp(pcm_end,gp)>0)) {
                    op_granpos_diff(&diff, gp, _target_gp);
                    /*We only actually use the current time if either
                     a) We can cut off at least half the range, or
                     b) We're seeking sufficiently close to the current position that
                     it's likely to be informative.
                     Otherwise it appears using the whole link range to estimate the
                     first seek location gives better results, on average.*/
                    if(diff < 0) {
                        OP_ASSERT(offset>=begin);
                        if(offset - begin >= end - begin >> 1 || diff > -OP_CUR_TIME_THRESH) {
                            best = begin = offset;
                            best_gp = pcm_start = gp;
                            /*If we have buffered data from a continued packet, remember the
                             offset of the previous page's start, so that if we do wind up
                             having to seek back here later, we can prime the stream with
                             the continued packet data.
                             With no continued packet, we remember the end of the page.*/
                            best_start = _of->os.body_returned < _of->os.body_fill ? _of->prev_page_offset : best;
                            /*If there's completed packets and data in the stream state,
                             prev_page_offset should always be set.*/
                            OP_ASSERT(best_start>=0);
                            /*Buffer any continued packet data starting from here.*/
                            buffering = 1;
                        }
                    }
                    else {
                        int64_t prev_page_gp;
                        /*We might get lucky and already have the packet with the target
                         buffered.
                         Worth checking.
                         For very small files (with all of the data in a single page,
                         generally 1 second or less), we can loop them continuously
                         without seeking at all.*/
                        op_granpos_add(&prev_page_gp, _of->op[0].granulepos,
                                       -op_get_packet_duration(_of->op[0].packet, _of->op[0].bytes));
                        if(op_granpos_cmp(prev_page_gp, _target_gp) <= 0) {
                            /*Don't call op_decode_clear(), because it will dump our
                             packets.*/
                            _of->op_pos = 0;
                            _of->od_buffer_size = 0;
                            _of->prev_packet_gp = prev_page_gp;
                            /*_of->prev_page_offset already points to the right place.*/
                            _of->ready_state = OP_STREAMSET;
                            return op_make_decode_ready(_of);
                        }
                        /*No such luck.
                         Check if we can cut off at least half the range, though.*/
                        if(offset - begin <= end - begin >> 1 || diff < OP_CUR_TIME_THRESH) {
                            /*We really want the page start here, but this will do.*/
                            end = boundary = offset;
                            pcm_end = gp;
                        }
                    }
                }
            }
        }

    }
    /*This code was originally based on the "new search algorithm by HB (Nicholas
     Vinen)" from libvorbisfile.
     It has been modified substantially since.*/
    op_decode_clear(_of);
    if(!buffering) ogg_stream_reset_serialno(&_of->os, serialno);
    _of->cur_link = _li;
    _of->ready_state = OP_STREAMSET;
    /*Initialize the interval size history.*/
    d2 = d1 = d0 = end - begin;
    force_bisect = 0;
    while(begin < end) {
        int64_t bisect;
        int64_t next_boundary;
        int32_t chunk_size;
        if(end - begin < OP_CHUNK_SIZE)
            bisect = begin;
        else {
            /*Update the interval size history.*/
            d0 = d1 >> 1;
            d1 = d2 >> 1;
            d2 = end - begin >> 1;
            if(force_bisect)
                bisect = begin + (end - begin >> 1);
            else {
                int64_t diff2;
                op_granpos_diff(&diff, _target_gp, pcm_start);
                op_granpos_diff(&diff2, pcm_end, pcm_start);
                /*Take a (pretty decent) guess.*/
                bisect = begin + op_rescale64(diff, diff2, end - begin) - OP_CHUNK_SIZE;
            }
            if(bisect - OP_CHUNK_SIZE < begin) bisect = begin;
            force_bisect = 0;
        }
        if(bisect != _of->offset) {
            /*Discard any buffered continued packet data.*/
            if(buffering) ogg_stream_reset(&_of->os);
            buffering = 0;
            page_offset = -1;
            ret = op_seek_helper(_of, bisect);
            if(ret < 0) return ret;
        }
        chunk_size = OP_CHUNK_SIZE;
        next_boundary = boundary;
        /*Now scan forward and figure out where we landed.
         In the ideal case, we will see a page with a granule position at or
         before our target, followed by a page with a granule position after our
         target (or the end of the search interval).
         Then we can just drop out and will have all of the data we need with no
         additional seeking.
         If we landed too far before, or after, we'll break out and do another
         bisection.*/
        while(begin < end) {
            page_offset = op_get_next_page(_of, &og, boundary);
            if(page_offset < 0) {
                if(page_offset < OP_FALSE) return (int) page_offset;
                /*There are no more pages in our interval from our stream with a valid
                 timestamp that start at position bisect or later.*/
                /*If we scanned the whole interval, we're done.*/
                if(bisect <= begin + 1)
                    end = begin;
                else {
                    /*Otherwise, back up one chunk.
                     First, discard any data from a continued packet.*/
                    if(buffering) ogg_stream_reset(&_of->os);
                    buffering = 0;
                    bisect = _max(bisect - chunk_size, begin);
                    ret = op_seek_helper(_of, bisect);
                    if(ret < 0) return ret;
                    /*Bump up the chunk size.*/
                    chunk_size = _min(2 * chunk_size, OP_CHUNK_SIZE_MAX);
                    /*If we did find a page from another stream or without a timestamp,
                     don't read past it.*/
                    boundary = next_boundary;
                }
            }
            else {
                int64_t gp;
                int has_packets;
                /*Save the offset of the first page we found after the seek, regardless
                 of the stream it came from or whether or not it has a timestamp.*/
                next_boundary = _min(page_offset, next_boundary);
                if(serialno != (uint32_t) ogg_page_serialno(&og)) continue;
                has_packets = ogg_page_packets(&og) > 0;
                /*Force the gp to -1 (as it should be per spec) if no packets end on
                 this page.
                 Otherwise we might get confused when we try to pull out a packet
                 with that timestamp and can't find it.*/
                gp = has_packets ? ogg_page_granulepos(&og) : -1;
                if(gp == -1) {
                    if(buffering) {
                        if(has_packets)
                            ogg_stream_pagein(&_of->os, &og);
                        else {
                            /*If packets did end on this page, but we still didn't have a
                             valid granule position (in violation of the spec!), stop
                             buffering continued packet data.
                             Otherwise we might continue past the packet we actually
                             wanted.*/
                            ogg_stream_reset(&_of->os);
                            buffering = 0;
                        }
                    }
                    continue;
                }
                if(op_granpos_cmp(gp, _target_gp) < 0) {
                    /*We found a page that ends before our target.
                     Advance to the raw offset of the next page.*/
                    begin = _of->offset;
                    if((op_granpos_cmp(pcm_start,gp)>0) || (op_granpos_cmp(pcm_end, gp) < 0)) {
                        /*Don't let pcm_start get out of range!
                         That could happen with an invalid timestamp.*/
                        break;
                    }
                    /*Save the byte offset of the end of the page with this granule
                     position.*/
                    best = best_start = begin;
                    /*Buffer any data from a continued packet, if necessary.
                     This avoids the need to seek back here if the next timestamp we
                     encounter while scanning forward lies after our target.*/
                    if(buffering) ogg_stream_reset(&_of->os);
                    if(op_page_continues(&og)) {
                        op_buffer_continued_data(_of, &og);
                        /*If we have a continued packet, remember the offset of this
                         page's start, so that if we do wind up having to seek back here
                         later, we can prime the stream with the continued packet data.
                         With no continued packet, we remember the end of the page.*/
                        best_start = page_offset;
                    }
                    /*Then force buffering on, so that if a packet starts (but does not
                     end) on the next page, we still avoid the extra seek back.*/
                    buffering = 1;
                    best_gp = pcm_start = gp;
                    op_granpos_diff(&diff, _target_gp, pcm_start);
                    /*If we're more than a second away from our target, break out and
                     do another bisection.*/
                    if(diff > 48000) break;
                    /*Otherwise, keep scanning forward (do NOT use begin+1).*/
                    bisect = begin;
                }
                else {
                    /*We found a page that ends after our target.*/
                    /*If we scanned the whole interval before we found it, we're done.*/
                    if(bisect <= begin + 1)
                        end = begin;
                    else {
                        end = bisect;
                        /*In later iterations, don't read past the first page we found.*/
                        boundary = next_boundary;
                        /*If we're not making much progress shrinking the interval size,
                         start forcing straight bisection to limit the worst case.*/
                        force_bisect = end - begin > d0 * 2;
                        /*Don't let pcm_end get out of range!
                         That could happen with an invalid timestamp.*/
                        if((op_granpos_cmp(pcm_end,gp)>0) && (op_granpos_cmp(pcm_start, gp) <= 0)) {
                            pcm_end = gp;
                        }
                        break;
                    }
                }
            }
        }
    }
    /*Found our page.*/
    OP_ASSERT(op_granpos_cmp(best_gp,pcm_start)>=0);
    /*Seek, if necessary.
     If we were buffering data from a continued packet, we should be able to
     continue to scan forward to get the rest of the data (even if
     page_offset==-1).
     Otherwise, we need to seek back to best_start.*/
    if(!buffering) {
        if(best_start != page_offset) {
            page_offset = -1;
            ret = op_seek_helper(_of, best_start);
            if(ret < 0) return ret;
        }
        if(best_start < best) {
            /*Retrieve the page at best_start, if we do not already have it.*/
            if(page_offset < 0) {
                page_offset = op_get_next_page(_of, &og, link->end_offset);
                if(page_offset < OP_FALSE) return (int) page_offset;
                if(page_offset != best_start) return OP_EBADLINK;
            }
            op_buffer_continued_data(_of, &og);
            page_offset = -1;
        }
    }
    /*Update prev_packet_gp to allow per-packet granule position assignment.*/
    _of->prev_packet_gp = best_gp;
    _of->prev_page_offset = best_start;
    ret = op_fetch_and_process_page(_of, page_offset < 0 ? NULL : &og, page_offset, 0, 1);
    if(ret < 0) return OP_EBADLINK;
    /*Verify result.*/
    if((op_granpos_cmp(_of->prev_packet_gp, _target_gp) > 0)) {
        return OP_EBADLINK;
    }
    /*Our caller will set cur_discard_count to handle pre-roll.*/
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int op_pcm_seek(OggOpusFile *_of, int64_t _pcm_offset) {
    const OggOpusLink_t *link;
    int64_t pcm_start;
    int64_t target_gp;
    int64_t prev_packet_gp;
    int64_t skip;
    int64_t diff;
    int op_count;
    int op_pos;
    int ret;
    int li;
    if(_of->ready_state<OP_OPENED) return OP_EINVAL;
    if(!_of->seekable) return OP_ENOSEEK;
    if(_pcm_offset < 0) return OP_EINVAL;
    target_gp = op_get_granulepos(_of, _pcm_offset, &li);
    if(target_gp == -1) return OP_EINVAL;
    link = _of->links + li;
    pcm_start = link->pcm_start;
    op_granpos_diff(&_pcm_offset, target_gp, pcm_start);

    /*For small (90 ms or less) forward seeks within the same link, just decode
     forward.
     This also optimizes the case of seeking to the current position.*/
    if(li == _of->cur_link && _of->ready_state >= OP_INITSET) {
        int64_t gp;
        gp = _of->prev_packet_gp;
        if(gp != -1) {
            int64_t discard_count;
            int nbuffered;
            nbuffered = _max(_of->od_buffer_size - _of->od_buffer_pos, 0);
            op_granpos_add(&gp, gp, -nbuffered);
            /*We do _not_ add cur_discard_count to gp.
             Otherwise the total amount to discard could grow without bound, and it
             would be better just to do a full seek.*/
            if(!op_granpos_diff(&discard_count, target_gp, gp)) {
                /*We use a threshold of 90 ms instead of 80, since 80 ms is the
                 _minimum_ we would have discarded after a full seek.
                 Assuming 20 ms frames (the default), we'd discard 90 ms on average.*/
                if(discard_count >= 0 && (discard_count < 90 * 48)) {
                    _of->cur_discard_count = (int32_t) discard_count;
                    return 0;
                }
            }
        }
    }
    ret = op_pcm_seek_page(_of, target_gp, li);
    if(ret < 0) return ret;
    /*Now skip samples until we actually get to our target.*/
    /*Figure out where we should skip to.*/
    if(_pcm_offset <= link->head.pre_skip)
        skip = 0;
    else
        skip = _max(_pcm_offset - 80 * 48, 0);
    OP_ASSERT(_pcm_offset-skip>=0); OP_ASSERT(_pcm_offset-skip<INT32_MAX-120*48);
    /*Skip packets until we find one with samples past our skip target.*/
    for(;;) {
        op_count = _of->op_count;
        prev_packet_gp = _of->prev_packet_gp;
        for(op_pos = _of->op_pos; op_pos < op_count; op_pos++) {
            int64_t cur_packet_gp;
            cur_packet_gp = _of->op[op_pos].granulepos;
            if(!op_granpos_diff(&diff,cur_packet_gp,pcm_start) && diff > skip) {
                break;
            }
            prev_packet_gp = cur_packet_gp;
        }
        _of->prev_packet_gp = prev_packet_gp;
        _of->op_pos = op_pos;
        if(op_pos < op_count) break;
        /*We skipped all the packets on this page.
         Fetch another.*/
        ret = op_fetch_and_process_page(_of, NULL, -1, 0, 1);
        if(ret < 0) return OP_EBADLINK;
    }
    /*We skipped too far, or couldn't get within 2 billion samples of the target.
     Either the timestamps were illegal or there was a hole in the data.*/
    if(op_granpos_diff(&diff, prev_packet_gp, pcm_start) || diff > skip || _pcm_offset - diff >= INT32_MAX) {
        return OP_EBADLINK;
    }
    /*TODO: If there are further holes/illegal timestamps, we still won't decode
     to the correct sample.
     However, at least op_pcm_tell() will report the correct value immediately
     after returning.*/
    _of->cur_discard_count = (int32_t) (_pcm_offset - diff);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int64_t op_raw_tell(const OggOpusFile *_of) {
    if(_of->ready_state<OP_OPENED) return OP_EINVAL;
    return _of->offset;
}
//----------------------------------------------------------------------------------------------------------------------
/*Convert a granule position from a given link to a PCM offset relative to the
 start of the whole stream.
 For unseekable sources, this gets reset to 0 at the beginning of each link.*/
static int64_t op_get_pcm_offset(const OggOpusFile *_of, int64_t _gp, int _li) {
    const OggOpusLink_t *links;
    int64_t pcm_offset;
    links = _of->links;
    OP_ASSERT(_li>=0&&_li<_of->nlinks);
    pcm_offset = links[_li].pcm_file_offset;
    if(_of->seekable && (op_granpos_cmp(_gp, links[_li].pcm_end) > 0)) {
        _gp = links[_li].pcm_end;
    }
    if(op_granpos_cmp(_gp, links[_li].pcm_start) > 0) {
        int64_t delta;
        if((op_granpos_diff(&delta, _gp, links[_li].pcm_start) < 0)) {
            /*This means an unseekable stream claimed to have a page from more than
             2 billion days after we joined.*/
            OP_ASSERT(!_of->seekable);
            return INT64_MAX;
        }
        if(delta < links[_li].head.pre_skip)
            delta = 0;
        else
            delta -= links[_li].head.pre_skip;
        /*In the seekable case, _gp was limited by pcm_end.
         In the unseekable case, pcm_offset should be 0.*/
        OP_ASSERT(pcm_offset<=INT64_MAX-delta);
        pcm_offset += delta;
    }
    return pcm_offset;
}
//----------------------------------------------------------------------------------------------------------------------
int64_t op_pcm_tell(const OggOpusFile *_of) {
    int64_t gp;
    int nbuffered;
    int li;
    if(_of->ready_state<OP_OPENED) return OP_EINVAL;
    gp = _of->prev_packet_gp;
    if(gp == -1) return 0;
    nbuffered = _max(_of->od_buffer_size - _of->od_buffer_pos, 0);
    op_granpos_add(&gp, gp, -nbuffered);
    li = _of->seekable ? _of->cur_link : 0;
    if(op_granpos_add(&gp, gp, _of->cur_discard_count) < 0) {
        gp = _of->links[li].pcm_end;
    }
    return op_get_pcm_offset(_of, gp, li);
}
//----------------------------------------------------------------------------------------------------------------------
void op_set_decode_callback(OggOpusFile *_of, op_decode_cb_func _decode_cb, void *_ctx) {
    _of->decode_cb = _decode_cb;
    _of->decode_cb_ctx = _ctx;
}
//----------------------------------------------------------------------------------------------------------------------
int op_set_gain_offset(OggOpusFile *_of, int _gain_type, int32_t _gain_offset_q8) {
    if(_gain_type != OP_HEADER_GAIN && _gain_type != OP_ALBUM_GAIN && _gain_type != OP_TRACK_GAIN
            && _gain_type != OP_ABSOLUTE_GAIN) {
        return OP_EINVAL;
    }
    _of->gain_type = _gain_type;
    /*The sum of header gain and track gain lies in the range [-65536,65534].
     These bounds allow the offset to set the final value to anywhere in the
     range [-32768,32767], which is what we'll clamp it to before applying.*/
    _of->gain_offset_q8 = OP_CLAMP(-98302, _gain_offset_q8, 98303);
    op_update_gain(_of);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
void op_set_dither_enabled(OggOpusFile *_of, int _enabled) {

    (void) _of;
    (void) _enabled;
}
//----------------------------------------------------------------------------------------------------------------------
/*Allocate the decoder scratch buffer.
 This is done lazily, since if the user provides large enough buffers, we'll
 never need it.*/
static int op_init_buffer(OggOpusFile *_of) {
    int nchannels_max;
    if(_of->seekable) {
        const OggOpusLink_t *links;
        int nlinks;
        int li;
        links = _of->links;
        nlinks = _of->nlinks;
        nchannels_max = 1;
        for(li = 0; li < nlinks; li++) {
            nchannels_max = _max(nchannels_max, links[li].head.channel_count);
        }
    }
    else
        nchannels_max = OP_NCHANNELS_MAX;
    _of->od_buffer = (op_sample*) malloc(sizeof(*_of->od_buffer) * nchannels_max * 120 * 48);
    if(_of->od_buffer == NULL) return OP_EFAULT;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Decode a single packet into the target buffer.*/
static int op_decode(OggOpusFile *_of, op_sample *_pcm, const ogg_packet *_op, int _nsamples, int _nchannels) {
    int ret;
    /*First we try using the application-provided decode callback.*/
    if(_of->decode_cb != NULL) {
        log_i("_nsamples %i, _nchannels %i _of->cur_link %i",_nsamples, _nchannels, _of->cur_link);
        ret = (*_of->decode_cb)(_of->decode_cb_ctx, _of->od, _pcm, _op, _nsamples, _nchannels, OP_DEC_FORMAT_SHORT,
                _of->cur_link);

    }
    else
        ret = OP_DEC_USE_DEFAULT;
    /*If the application didn't want to handle decoding, do it ourselves.*/
    if(ret == OP_DEC_USE_DEFAULT) {
        ret = opus_multistream_decode(_of->od, _op->packet, _op->bytes, _pcm, _nsamples, 0);
        OP_ASSERT(ret < 0 || ret == _nsamples);
    }
    /*If the application returned a positive value other than 0 or
     OP_DEC_USE_DEFAULT, fail.*/
    else if(ret > 0) return OP_EBADPACKET;
    if(ret < 0) return OP_EBADPACKET;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
/*Read more samples from the stream, using the same API as op_read() or op_read_float().*/
static int op_read_native(OggOpusFile *_of, op_sample *_pcm, int _buf_size, int *_li) {

    if(_of->ready_state<OP_OPENED) return OP_EINVAL;
    for(;;) {
        int ret;
        if(_of->ready_state>=OP_INITSET) {
            int nchannels;
            int od_buffer_pos;
            int nsamples;
            int op_pos;
            nchannels = _of->links[_of->seekable ? _of->cur_link : 0].head.channel_count;
            od_buffer_pos = _of->od_buffer_pos;
            nsamples = _of->od_buffer_size - od_buffer_pos;
            /*If we have buffered samples, return them.*/
            if(nsamples > 0) {
                if(nsamples * nchannels > _buf_size) nsamples = _buf_size / nchannels;
                OP_ASSERT(_pcm!=NULL||nsamples<=0);
                /*Check nsamples again so we don't pass NULL to memcpy() if _buf_size
                 is zero.
                 That would technically be undefined behavior, even if the number of
                 bytes to copy were zero.*/
                if(nsamples > 0) {
                    memcpy(_pcm, _of->od_buffer + nchannels * od_buffer_pos, sizeof(*_pcm) * nchannels * nsamples);
                    od_buffer_pos += nsamples;
                    _of->od_buffer_pos = od_buffer_pos;
                }
                if(_li != NULL) *_li = _of->cur_link;
                return nsamples;
            }
            /*If we have buffered packets, decode one.*/
            op_pos = _of->op_pos;
            if(op_pos < _of->op_count) {
                const ogg_packet *pop;
                int64_t diff;
                int32_t cur_discard_count;
                int duration;
                int trimmed_duration;
                pop = _of->op + op_pos++;
                _of->op_pos = op_pos;
                cur_discard_count = _of->cur_discard_count;
                duration = op_get_packet_duration(pop->packet, pop->bytes);
                /*We don't buffer packets with an invalid TOC sequence.*/
                OP_ASSERT(duration>0);
                trimmed_duration = duration;
                /*Perform end-trimming.*/
                if(pop->e_o_s) {
                    if(op_granpos_cmp(pop->granulepos, _of->prev_packet_gp) <= 0) {
                        trimmed_duration = 0;
                    }
                    else if(!op_granpos_diff(&diff, pop->granulepos, _of->prev_packet_gp)) {
                        trimmed_duration = (int) _min(diff, trimmed_duration);
                    }
                }
                _of->prev_packet_gp = pop->granulepos;
                if(duration * nchannels > _buf_size) {
                    op_sample *buf;
                    /*If the user's buffer is too small, decode into a scratch buffer.*/
                    buf = _of->od_buffer;
                    if(buf==NULL) {
                        ret = op_init_buffer(_of);
                        if(ret < 0) return ret;
                        buf = _of->od_buffer;
                    }
                    ret = op_decode(_of, buf, pop, duration, nchannels);
                    if(ret < 0) return ret;
                    /*Perform pre-skip/pre-roll.*/
                    od_buffer_pos = (int) _min(trimmed_duration, cur_discard_count);
                    cur_discard_count -= od_buffer_pos;
                    _of->cur_discard_count = cur_discard_count;
                    _of->od_buffer_pos = od_buffer_pos;
                    _of->od_buffer_size = trimmed_duration;
                    /*Update bitrate tracking based on the actual samples we used from
                     what was decoded.*/
                    _of->bytes_tracked += pop->bytes;
                    _of->samples_tracked += trimmed_duration - od_buffer_pos;
                }
                else {
                    OP_ASSERT(_pcm!=NULL);
                    /*Otherwise decode directly into the user's buffer.*/
                    ret = op_decode(_of, _pcm, pop, duration, nchannels);
                    if(ret < 0) return ret;
                    if(trimmed_duration > 0) {
                        /*Perform pre-skip/pre-roll.*/
                        od_buffer_pos = (int) _min(trimmed_duration, cur_discard_count);
                        cur_discard_count -= od_buffer_pos;
                        _of->cur_discard_count = cur_discard_count;
                        trimmed_duration -= od_buffer_pos;
                        if((trimmed_duration>0) && (od_buffer_pos > 0)) {
                            memmove(_pcm, _pcm + od_buffer_pos * nchannels,
                                    sizeof(*_pcm) * trimmed_duration * nchannels);
                        }
                        /*Update bitrate tracking based on the actual samples we used from
                         what was decoded.*/
                        _of->bytes_tracked += pop->bytes;
                        _of->samples_tracked += trimmed_duration;
                        if(trimmed_duration > 0) {
                            if(_li != NULL) *_li = _of->cur_link;
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
        ret = op_fetch_and_process_page(_of, NULL, -1, 1, 0);
        if(ret==OP_EOF) {
            if(_li != NULL) *_li = _of->cur_link;
            return 0;
        }
        if(ret < 0) return ret;
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*A generic filter to apply to the decoded audio data.
 _src is non-const because we will destructively modify the contents of the
 source buffer that we consume in some cases.*/
typedef int (*op_read_filter_func)(OggOpusFile *_of, void *_dst, int _dst_sz, op_sample *_src, int _nsamples,
        int _nchannels);
//----------------------------------------------------------------------------------------------------------------------
/*Decode some samples and then apply a custom filter to them.
 This is used to convert to different output formats.*/
static int op_filter_read_native(OggOpusFile *_of, void *_dst, int _dst_sz, op_read_filter_func _filter, int *_li) {
    int ret;
    /*Ensure we have some decoded samples in our buffer.*/
    ret = op_read_native(_of, NULL, 0, _li);
    /*Now apply the filter to them.*/
    if((ret>=0) && (_of->ready_state>=OP_INITSET)) {
        int od_buffer_pos;
        od_buffer_pos = _of->od_buffer_pos;
        ret = _of->od_buffer_size - od_buffer_pos;
        if(ret > 0) {
            int nchannels;
            nchannels = _of->links[_of->seekable ? _of->cur_link : 0].head.channel_count;
            ret = (*_filter)(_of, _dst, _dst_sz, _of->od_buffer + nchannels * od_buffer_pos, ret, nchannels);
            OP_ASSERT(ret>=0); OP_ASSERT(ret<=_of->od_buffer_size-od_buffer_pos);
            od_buffer_pos += ret;
            _of->od_buffer_pos = od_buffer_pos;
        }
    }
    return ret;
}

//int op_read(OggOpusFile *_of, int16_t *_pcm, int _buf_size, int *_li) {
//    return op_read_native(_of, _pcm, _buf_size, _li);
//}
//----------------------------------------------------------------------------------------------------------------------
static int op_stereo_filter(OggOpusFile *_of, void *_dst, int _dst_sz, op_sample *_src, int _nsamples, int _nchannels) {
    (void) _of;
    _nsamples = _min(_nsamples, _dst_sz >> 1);
    if(_nchannels == 2)
        memcpy(_dst, _src, _nsamples * 2 * sizeof(*_src));
    else {
        int16_t *dst;
        int i;
        dst = (int16_t*) _dst;
        if(_nchannels == 1) {
            for(i = 0; i < _nsamples; i++)
                dst[2 * i + 0] = dst[2 * i + 1] = _src[i];
        }
        else {

            // noop, removed for RAM savings
        }
    }
    return _nsamples;
}
//----------------------------------------------------------------------------------------------------------------------
int op_read_stereo(OggOpusFile *_of, int16_t *_pcm, int _buf_size) {

    return op_filter_read_native(_of, _pcm, _buf_size, op_stereo_filter, NULL);
}
//----------------------------------------------------------------------------------------------------------------------
unsigned op_parse_uint16le(const unsigned char *_data) {
    return _data[0] | _data[1] << 8;
}
//----------------------------------------------------------------------------------------------------------------------
int op_parse_int16le(const unsigned char *_data) {
    int ret;
    ret = _data[0] | _data[1] << 8;
    return (ret ^ 0x8000) - 0x8000;
}
//----------------------------------------------------------------------------------------------------------------------
uint32_t op_parse_uint32le(const unsigned char *_data) {
    return _data[0] | (uint32_t) _data[1] << 8 | (uint32_t) _data[2] << 16 | (uint32_t) _data[3] << 24;
}

uint32_t op_parse_uint32be(const unsigned char *_data) {
    return _data[3] | (uint32_t) _data[2] << 8 | (uint32_t) _data[1] << 16 | (uint32_t) _data[0] << 24;
}
//----------------------------------------------------------------------------------------------------------------------
/*A version of strncasecmp() that is guaranteed to only ignore the case of
 ASCII characters.*/
int op_strncasecmp(const char *_a, const char *_b, int _n) {
    int i;
    for(i = 0; i < _n; i++) {
        int a;
        int b;
        int d;
        a = _a[i];
        b = _b[i];
        if(a >= 'a' && a <= 'z') a -= 'a' - 'A';
        if(b >= 'a' && b <= 'z') b -= 'a' - 'A';
        d = a - b;
        if(d) return d;
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_head_parse(OpusHead_t *_head, const unsigned char *_data, size_t _len) {
    OpusHead_t head;
    if(_len < 8) return OP_ENOTFORMAT;
    if(memcmp(_data, "OpusHead", 8) != 0) return OP_ENOTFORMAT;
    if(_len < 9) return OP_EBADHEADER;
    head.version = _data[8];
    if(head.version > 15) return OP_EVERSION;
    if(_len < 19) return OP_EBADHEADER;
    head.channel_count = _data[9];
    head.pre_skip = op_parse_uint16le(_data + 10);
    head.input_sample_rate = op_parse_uint32le(_data + 12);
    head.output_gain = op_parse_int16le(_data + 16);
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
void opus_tags_init(OpusTags_t *_tags) {
    memset(_tags, 0, sizeof(*_tags));
}
//----------------------------------------------------------------------------------------------------------------------
void opus_tags_clear(OpusTags_t *_tags) {
    int ncomments;
    int ci;
    ncomments = _tags->comments;
    if(_tags->user_comments != NULL)
        ncomments++;
    else {
        OP_ASSERT(ncomments==0);
    }
    for(ci = ncomments; ci-- > 0;)
        free(_tags->user_comments[ci]);
    free(_tags->user_comments);
    free(_tags->comment_lengths);
    free(_tags->vendor);
}
//----------------------------------------------------------------------------------------------------------------------
/*Ensure there's room for up to _ncomments comments.*/
int op_tags_ensure_capacity(OpusTags_t *_tags, size_t _ncomments) {
    char **user_comments;
    int *comment_lengths;
    int cur_ncomments;
    size_t size;
    if(_ncomments >= (size_t) INT_MAX) return OP_EFAULT;
    size = sizeof(*_tags->comment_lengths) * (_ncomments + 1);
    if(size / sizeof(*_tags->comment_lengths) != _ncomments + 1) return OP_EFAULT;
    cur_ncomments = _tags->comments;
    /*We only support growing.
     Trimming requires cleaning up the allocated strings in the old space, and
     is best handled separately if it's ever needed.*/
    OP_ASSERT(_ncomments>=(size_t)cur_ncomments);
    comment_lengths = (int*) realloc(_tags->comment_lengths, size);
    if(comment_lengths == NULL) return OP_EFAULT;
    if(_tags->comment_lengths == NULL) {
        OP_ASSERT(cur_ncomments==0);
        comment_lengths[cur_ncomments] = 0;
    }
    comment_lengths[_ncomments] = comment_lengths[cur_ncomments];
    _tags->comment_lengths = comment_lengths;
    size = sizeof(*_tags->user_comments) * (_ncomments + 1);
    if(size / sizeof(*_tags->user_comments) != _ncomments + 1) return OP_EFAULT;
    user_comments = (char**) realloc(_tags->user_comments, size);
    if(user_comments == NULL) return OP_EFAULT;
    if(_tags->user_comments == NULL) {
        OP_ASSERT(cur_ncomments==0);
        user_comments[cur_ncomments] = NULL;
    }
    user_comments[_ncomments] = user_comments[cur_ncomments];
    _tags->user_comments = user_comments;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Duplicate a (possibly non-NUL terminated) string with a known length.*/
char* op_strdup_with_len(const char *_s, size_t _len) {
    size_t size;
    char *ret;
    size = sizeof(*ret) * (_len + 1);
    if(size < _len) return NULL;
    ret = (char*) malloc(size);
    if(ret != NULL) {
        ret = (char*) memcpy(ret, _s, sizeof(*ret) * _len);
        ret[_len] = '\0';
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
/*The actual implementation of opus_tags_parse().
 Unlike the public API, this function requires _tags to already be
 initialized, modifies its contents before success is guaranteed, and assumes
 the caller will clear it on error.*/
int opus_tags_parse_impl(OpusTags_t *_tags, const unsigned char *_data, size_t _len) {
    uint32_t count;
    size_t len;
    int ncomments;
    int ci;
    len = _len;
    if(len < 8) return OP_ENOTFORMAT;
    if(memcmp(_data, "OpusTags", 8) != 0) return OP_ENOTFORMAT;
    if(len < 16) return OP_EBADHEADER;
    _data += 8;
    len -= 8;
    count = op_parse_uint32le(_data);
    _data += 4;
    len -= 4;
    if(count > len) return OP_EBADHEADER;
    if(_tags != NULL) {
        _tags->vendor = op_strdup_with_len((char*) _data, count);
        if(_tags->vendor == NULL) return OP_EFAULT;
    }
    _data += count;
    len -= count;
    if(len < 4) return OP_EBADHEADER;
    count = op_parse_uint32le(_data);
    _data += 4;
    len -= 4;
    /*Check to make sure there's minimally sufficient data left in the packet.*/
    if(count > len >> 2) return OP_EBADHEADER;
    /*Check for overflow (the API limits this to an int).*/
    if(count > (uint32_t) INT_MAX - 1) return OP_EFAULT;
    if(_tags != NULL) {
        int ret;
        ret = op_tags_ensure_capacity(_tags, count);
        if(ret < 0) return ret;
    }
    ncomments = (int) count;
    for(ci = 0; ci < ncomments; ci++) {
        /*Check to make sure there's minimally sufficient data left in the packet.*/
        if((size_t) (ncomments - ci) > len >> 2) return OP_EBADHEADER;
        count = op_parse_uint32le(_data);
        _data += 4;
        len -= 4;
        if(count > len) return OP_EBADHEADER;
        /*Check for overflow (the API limits this to an int).*/
        if(count > (uint32_t) INT_MAX) return OP_EFAULT;
        if(_tags != NULL) {
            _tags->user_comments[ci] = op_strdup_with_len((char*) _data, count);
            if(_tags->user_comments[ci] == NULL) return OP_EFAULT;
            _tags->comment_lengths[ci] = (int) count;
            _tags->comments = ci + 1;
            /*Needed by opus_tags_clear() if we fail before parsing the (optional)
             binary metadata.*/
            _tags->user_comments[ci + 1] = NULL;
        }
        _data += count;
        len -= count;
    }
    if(len > 0 && (_data[0] & 1)) {
        if(len > (uint32_t) INT_MAX) return OP_EFAULT;
        if(_tags != NULL) {
            _tags->user_comments[ncomments] = (char*) malloc(len);
            if(_tags->user_comments[ncomments] == NULL) return OP_EFAULT;
            memcpy(_tags->user_comments[ncomments], _data, len);
            _tags->comment_lengths[ncomments] = (int) len;
        }
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_parse(OpusTags_t *_tags, const unsigned char *_data, size_t _len) {
    if(_tags != NULL) {
        OpusTags_t tags;
        int ret;
        opus_tags_init(&tags);
        ret = opus_tags_parse_impl(&tags, _data, _len);
        if(ret < 0)
            opus_tags_clear(&tags);
        else
            *_tags = *&tags;
        return ret;
    }
    else
        return opus_tags_parse_impl(NULL, _data, _len);
}
//----------------------------------------------------------------------------------------------------------------------
/*The actual implementation of opus_tags_copy().
 Unlike the public API, this function requires _dst to already be
 initialized, modifies its contents before success is guaranteed, and assumes
 the caller will clear it on error.*/
int opus_tags_copy_impl(OpusTags_t *_dst, const OpusTags_t *_src) {
    char *vendor;
    int ncomments;
    int ret;
    int ci;
    vendor = _src->vendor;
    _dst->vendor = op_strdup_with_len(vendor, strlen(vendor));
    if(_dst->vendor == NULL) return OP_EFAULT;
    ncomments = _src->comments;
    ret = op_tags_ensure_capacity(_dst, ncomments);
    if(ret < 0) return ret;
    for(ci = 0; ci < ncomments; ci++) {
        int len;
        len = _src->comment_lengths[ci];
        OP_ASSERT(len>=0);
        _dst->user_comments[ci] = op_strdup_with_len(_src->user_comments[ci], len);
        if(_dst->user_comments[ci] == NULL) return OP_EFAULT;
        _dst->comment_lengths[ci] = len;
        _dst->comments = ci + 1;
    }
    if(_src->comment_lengths != NULL) {
        int len;
        len = _src->comment_lengths[ncomments];
        if(len > 0) {
            _dst->user_comments[ncomments] = (char*) malloc(len);
            if(_dst->user_comments[ncomments] == NULL) return OP_EFAULT;
            memcpy(_dst->user_comments[ncomments], _src->user_comments[ncomments], len);
            _dst->comment_lengths[ncomments] = len;
        }
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_copy(OpusTags_t *_dst, const OpusTags_t *_src) {
    OpusTags_t dst;
    int ret;
    opus_tags_init(&dst);
    ret = opus_tags_copy_impl(&dst, _src);
    if(ret < 0)
        opus_tags_clear(&dst);
    else
        *_dst = *&dst;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_add(OpusTags_t *_tags, const char *_tag, const char *_value) {
    char *comment;
    size_t tag_len;
    size_t value_len;
    int ncomments;
    int ret;
    ncomments = _tags->comments;
    ret = op_tags_ensure_capacity(_tags, ncomments + 1);
    if(ret < 0) return ret;
    tag_len = strlen(_tag);
    value_len = strlen(_value);
    /*+2 for '=' and '\0'.*/
    if(tag_len + value_len < tag_len) return OP_EFAULT;
    if(tag_len + value_len > (size_t) INT_MAX - 2) return OP_EFAULT;
    comment = (char*) malloc(sizeof(*comment) * (tag_len + value_len + 2));
    if(comment == NULL) return OP_EFAULT;
    memcpy(comment, _tag, sizeof(*comment) * tag_len);
    comment[tag_len] = '=';
    memcpy(comment + tag_len + 1, _value, sizeof(*comment) * (value_len + 1));
    _tags->user_comments[ncomments] = comment;
    _tags->comment_lengths[ncomments] = (int) (tag_len + value_len + 1);
    _tags->comments = ncomments + 1;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_add_comment(OpusTags_t *_tags, const char *_comment) {
    char *comment;
    int comment_len;
    int ncomments;
    int ret;
    ncomments = _tags->comments;
    ret = op_tags_ensure_capacity(_tags, ncomments + 1);
    if(ret < 0) return ret;
    comment_len = (int) strlen(_comment);
    comment = op_strdup_with_len(_comment, comment_len);
    if(comment == NULL) return OP_EFAULT;
    _tags->user_comments[ncomments] = comment;
    _tags->comment_lengths[ncomments] = comment_len;
    _tags->comments = ncomments + 1;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_set_binary_suffix(OpusTags_t *_tags, const unsigned char *_data, int _len) {
    unsigned char *binary_suffix_data;
    int ncomments;
    int ret;
    if((_len < 0) || (((_len > 0) && (_data == NULL)) || !(_data[0] & 1))) return OP_EINVAL;
    ncomments = _tags->comments;
    ret = op_tags_ensure_capacity(_tags, ncomments);
    if(ret < 0) return ret;
    binary_suffix_data = (unsigned char*) realloc(_tags->user_comments[ncomments], _len);
    if(binary_suffix_data == NULL) return OP_EFAULT;
    memcpy(binary_suffix_data, _data, _len);
    _tags->user_comments[ncomments] = (char*) binary_suffix_data;
    _tags->comment_lengths[ncomments] = _len;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tagcompare(const char *_tag_name, const char *_comment) {
    size_t tag_len;
    tag_len = strlen(_tag_name);
    if(tag_len > (size_t) INT_MAX) return -1;
    return opus_tagncompare(_tag_name, (int) tag_len, _comment);
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tagncompare(const char *_tag_name, int _tag_len, const char *_comment) {
    int ret;
    OP_ASSERT(_tag_len>=0);
    ret = op_strncasecmp(_tag_name, _comment, _tag_len);
    return ret ? ret : '=' - _comment[_tag_len];
}
//----------------------------------------------------------------------------------------------------------------------
const char* opus_tags_query(const OpusTags_t *_tags, const char *_tag, int _count) {
    char **user_comments;
    size_t tag_len;
    int found;
    int ncomments;
    int ci;
    tag_len = strlen(_tag);
    if(tag_len > (size_t) INT_MAX) return NULL;
    ncomments = _tags->comments;
    user_comments = _tags->user_comments;
    found = 0;
    for(ci = 0; ci < ncomments; ci++) {
        if(!opus_tagncompare(_tag, (int) tag_len, user_comments[ci])) {
            /*We return a pointer to the data, not a copy.*/
            if(_count == found++) return user_comments[ci] + tag_len + 1;
        }
    }
    /*Didn't find anything.*/
    return NULL;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_query_count(const OpusTags_t *_tags, const char *_tag) {
    char **user_comments;
    size_t tag_len;
    int found;
    int ncomments;
    int ci;
    tag_len = strlen(_tag);
    if(tag_len > (size_t) INT_MAX) return 0;
    ncomments = _tags->comments;
    user_comments = _tags->user_comments;
    found = 0;
    for(ci = 0; ci < ncomments; ci++) {
        if(!opus_tagncompare(_tag, (int) tag_len, user_comments[ci])) found++;
    }
    return found;
}
//----------------------------------------------------------------------------------------------------------------------
const unsigned char* opus_tags_get_binary_suffix(const OpusTags_t *_tags, int *_len) {
    int ncomments;
    int len;
    ncomments = _tags->comments;
    len = _tags->comment_lengths == NULL ? 0 : _tags->comment_lengths[ncomments];
    *_len = len;
    OP_ASSERT(len==0||_tags->user_comments!=NULL);
    return len > 0 ? (const unsigned char*) _tags->user_comments[ncomments] : NULL;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_get_gain(const OpusTags_t *_tags, int *_gain_q8, const char *_tag_name, size_t _tag_len) {
    char **comments;
    int ncomments;
    int ci;
    comments = _tags->user_comments;
    ncomments = _tags->comments;
    /*Look for the first valid tag with the name _tag_name and use that.*/
    for(ci = 0; ci < ncomments; ci++) {
        OP_ASSERT(_tag_len<=(size_t)INT_MAX);
        if(opus_tagncompare(_tag_name, (int) _tag_len, comments[ci]) == 0) {
            char *p;
            int32_t gain_q8;
            int negative;
            p = comments[ci] + _tag_len + 1;
            negative = 0;
            if(*p == '-') {
                negative = -1;
                p++;
            }
            else if(*p == '+') p++;
            gain_q8 = 0;
            while(*p >= '0' && *p <= '9') {
                gain_q8 = 10 * gain_q8 + *p - '0';
                if(gain_q8 > 32767 - negative) break;
                p++;
            }
            /*This didn't look like a signed 16-bit decimal integer.
             Not a valid gain tag.*/
            if(*p != '\0') continue;
            *_gain_q8 = (int) (gain_q8 + negative ^ negative);
            return 0;
        }
    }
    return OP_FALSE;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_get_album_gain(const OpusTags_t *_tags, int *_gain_q8) {
    return opus_tags_get_gain(_tags, _gain_q8, "R128_ALBUM_GAIN", 15);
}
//----------------------------------------------------------------------------------------------------------------------
int opus_tags_get_track_gain(const OpusTags_t *_tags, int *_gain_q8) {
    return opus_tags_get_gain(_tags, _gain_q8, "R128_TRACK_GAIN", 15);
}
//----------------------------------------------------------------------------------------------------------------------
int op_is_jpeg(const unsigned char *_buf, size_t _buf_sz) {
    return _buf_sz >= 3 && memcmp(_buf, "\xFF\xD8\xFF", 3) == 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Tries to extract the width, height, bits per pixel, and palette size of a
 JPEG.
 On failure, simply leaves its outputs unmodified.*/
void op_extract_jpeg_params(const unsigned char *_buf, size_t _buf_sz, uint32_t *_width, uint32_t *_height,
        uint32_t *_depth, uint32_t *_colors, int *_has_palette) {
    if(op_is_jpeg(_buf, _buf_sz)) {
        size_t offs;
        offs = 2;
        for(;;) {
            size_t segment_len;
            int marker;
            while(offs < _buf_sz && _buf[offs] != 0xFF)
                offs++;
            while(offs < _buf_sz && _buf[offs] == 0xFF)
                offs++;
            marker = _buf[offs];
            offs++;
            /*If we hit EOI* (end of image), or another SOI* (start of image),
             or SOS (start of scan), then stop now.*/
            if(offs >= _buf_sz || (marker >= 0xD8 && marker <= 0xDA))
                break;
            /*RST* (restart markers): skip (no segment length).*/
            else if(marker >= 0xD0 && marker <= 0xD7) continue;
            /*Read the length of the marker segment.*/
            if(_buf_sz - offs < 2) break;
            segment_len = _buf[offs] << 8 | _buf[offs + 1];
            if(segment_len < 2 || _buf_sz - offs < segment_len) break;
            if(marker == 0xC0 || (marker > 0xC0 && marker < 0xD0 && (marker & 3) != 0)) {
                /*Found a SOFn (start of frame) marker segment:*/
                if(segment_len >= 8) {
                    *_height = _buf[offs + 3] << 8 | _buf[offs + 4];
                    *_width = _buf[offs + 5] << 8 | _buf[offs + 6];
                    *_depth = _buf[offs + 2] * _buf[offs + 7];
                    *_colors = 0;
                    *_has_palette = 0;
                }
                break;
            }
            /*Other markers: skip the whole marker segment.*/
            offs += segment_len;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------
int op_is_png(const unsigned char *_buf, size_t _buf_sz) {
    return _buf_sz >= 8 && memcmp(_buf, "\x89PNG\x0D\x0A\x1A\x0A", 8) == 0;
}
//----------------------------------------------------------------------------------------------------------------------
/*Tries to extract the width, height, bits per pixel, and palette size of a
 PNG.
 On failure, simply leaves its outputs unmodified.*/
void op_extract_png_params(const unsigned char *_buf, size_t _buf_sz, uint32_t *_width, uint32_t *_height,
        uint32_t *_depth, uint32_t *_colors, int *_has_palette) {
    if(op_is_png(_buf, _buf_sz)) {
        size_t offs;
        offs = 8;
        while(_buf_sz - offs >= 12) {
            uint32_t chunk_len;
            chunk_len = op_parse_uint32be(_buf + offs);
            if(chunk_len > _buf_sz - (offs + 12))
                break;
            else if(chunk_len == 13 && memcmp(_buf + offs + 4, "IHDR", 4) == 0) {
                int color_type;
                *_width = op_parse_uint32be(_buf + offs + 8);
                *_height = op_parse_uint32be(_buf + offs + 12);
                color_type = _buf[offs + 17];
                if(color_type == 3) {
                    *_depth = 24;
                    *_has_palette = 1;
                }
                else {
                    int sample_depth;
                    sample_depth = _buf[offs + 16];
                    if(color_type == 0)
                        *_depth = sample_depth;
                    else if(color_type == 2)
                        *_depth = sample_depth * 3;
                    else if(color_type == 4)
                        *_depth = sample_depth * 2;
                    else if(color_type == 6) *_depth = sample_depth * 4;
                    *_colors = 0;
                    *_has_palette = 0;
                    break;
                }
            }
            else if(*_has_palette > 0 && memcmp(_buf + offs + 4, "PLTE", 4) == 0) {
                *_colors = chunk_len / 3;
                break;
            }
            offs += 12 + chunk_len;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------
int op_is_gif(const unsigned char *_buf, size_t _buf_sz) {
    return _buf_sz >= 6 && (memcmp(_buf, "GIF87a", 6) == 0 || memcmp(_buf, "GIF89a", 6) == 0);
}
//----------------------------------------------------------------------------------------------------------------------
/*Tries to extract the width, height, bits per pixel, and palette size of a
 GIF.
 On failure, simply leaves its outputs unmodified.*/
void op_extract_gif_params(const unsigned char *_buf, size_t _buf_sz, uint32_t *_width, uint32_t *_height,
        uint32_t *_depth, uint32_t *_colors, int *_has_palette) {
    if(op_is_gif(_buf, _buf_sz) && _buf_sz >= 14) {
        *_width = _buf[6] | _buf[7] << 8;
        *_height = _buf[8] | _buf[9] << 8;
        /*libFLAC hard-codes the depth to 24.*/
        *_depth = 24;
        *_colors = 1 << ((_buf[10] & 7) + 1);
        *_has_palette = 1;
    }
}
//----------------------------------------------------------------------------------------------------------------------
/*The actual implementation of opus_picture_tag_parse().
 Unlike the public API, this function requires _pic to already be
 initialized, modifies its contents before success is guaranteed, and assumes
 the caller will clear it on error.*/
int opus_picture_tag_parse_impl(OpusPictureTag_t *_pic, const char *_tag, unsigned char *_buf, size_t _buf_sz,
        size_t _base64_sz) {
    int32_t picture_type;
    uint32_t mime_type_length;
    char *mime_type;
    uint32_t description_length;
    char *description;
    uint32_t width;
    uint32_t height;
    uint32_t depth;
    uint32_t colors;
    uint32_t data_length;
    uint32_t file_width;
    uint32_t file_height;
    uint32_t file_depth;
    uint32_t file_colors;
    int format;
    int has_palette;
    int colors_set;
    size_t i;
    /*Decode the BASE64 data.*/
    OP_ASSERT(_base64_sz>=11);
    for(i = 0; i < _base64_sz; i++) {
        uint32_t value;
        int j;
        value = 0;
        for(j = 0; j < 4; j++) {
            unsigned c;
            unsigned d;
            c = (unsigned char) _tag[4 * i + j];
            if(c == '+')
                d = 62;
            else if(c == '/')
                d = 63;
            else if(c >= '0' && c <= '9')
                d = 52 + c - '0';
            else if(c >= 'a' && c <= 'z')
                d = 26 + c - 'a';
            else if(c >= 'A' && c <= 'Z')
                d = c - 'A';
            else if(c == '=' && 3 * i + j > _buf_sz)
                d = 0;
            else
                return OP_ENOTFORMAT;
            value = value << 6 | d;
        }
        _buf[3 * i] = (unsigned char) (value >> 16);
        if(3 * i + 1 < _buf_sz) {
            _buf[3 * i + 1] = (unsigned char) (value >> 8);
            if(3 * i + 2 < _buf_sz) _buf[3 * i + 2] = (unsigned char) value;
        }
    }
    i = 0;
    picture_type = op_parse_uint32be(_buf + i);
    i += 4;
    /*Extract the MIME type.*/
    mime_type_length = op_parse_uint32be(_buf + i);
    i += 4;
    if(mime_type_length > _buf_sz - 32) return OP_ENOTFORMAT;
    mime_type = (char*) malloc(sizeof(*_pic->mime_type) * (mime_type_length + 1));
    if(mime_type == NULL) return OP_EFAULT;
    memcpy(mime_type, _buf + i, sizeof(*mime_type) * mime_type_length);
    mime_type[mime_type_length] = '\0';
    _pic->mime_type = mime_type;
    i += mime_type_length;
    /*Extract the description string.*/
    description_length = op_parse_uint32be(_buf + i);
    i += 4;
    if(description_length > _buf_sz - mime_type_length - 32) return OP_ENOTFORMAT;
    description = (char*) malloc(sizeof(*_pic->mime_type) * (description_length + 1));
    if(description == NULL) return OP_EFAULT;
    memcpy(description, _buf + i, sizeof(*description) * description_length);
    description[description_length] = '\0';
    _pic->description = description;
    i += description_length;
    /*Extract the remaining fields.*/
    width = op_parse_uint32be(_buf + i);
    i += 4;
    height = op_parse_uint32be(_buf + i);
    i += 4;
    depth = op_parse_uint32be(_buf + i);
    i += 4;
    colors = op_parse_uint32be(_buf + i);
    i += 4;
    /*If one of these is set, they all must be, but colors==0 is a valid value.*/
    colors_set = width != 0 || height != 0 || depth != 0 || colors != 0;
    if((width == 0 || height == 0 || depth == 0) && colors_set) return OP_ENOTFORMAT;
    data_length = op_parse_uint32be(_buf + i);
    i += 4;
    if(data_length > _buf_sz - i) return OP_ENOTFORMAT;
    /*Trim extraneous data so we don't copy it below.*/
    _buf_sz = i + data_length;
    /*Attempt to determine the image format.*/
    format = OP_PIC_FORMAT_UNKNOWN;
    if(mime_type_length == 3 && strcmp(mime_type, "-->") == 0) {
        format = OP_PIC_FORMAT_URL;
        /*Picture type 1 must be a 32x32 PNG.*/
        if(picture_type == 1 && (width != 0 || height != 0) && (width != 32 || height != 32)) {
            return OP_ENOTFORMAT;
        }
        /*Append a terminating NUL for the convenience of our callers.*/
        _buf[_buf_sz++] = '\0';
    }
    else {
        if(mime_type_length == 10 && op_strncasecmp(mime_type, "image/jpeg", mime_type_length) == 0) {
            if(op_is_jpeg(_buf + i, data_length)) format = OP_PIC_FORMAT_JPEG;
        }
        else if(mime_type_length == 9 && op_strncasecmp(mime_type, "image/png", mime_type_length) == 0) {
            if(op_is_png(_buf + i, data_length)) format = OP_PIC_FORMAT_PNG;
        }
        else if(mime_type_length == 9 && op_strncasecmp(mime_type, "image/gif", mime_type_length) == 0) {
            if(op_is_gif(_buf + i, data_length)) format = OP_PIC_FORMAT_GIF;
        }
        else if(mime_type_length == 0
                || (mime_type_length == 6 && op_strncasecmp(mime_type, "image/", mime_type_length) == 0)) {
            if(op_is_jpeg(_buf + i, data_length))
                format = OP_PIC_FORMAT_JPEG;
            else if(op_is_png(_buf + i, data_length))
                format = OP_PIC_FORMAT_PNG;
            else if(op_is_gif(_buf + i, data_length)) format = OP_PIC_FORMAT_GIF;
        }
        file_width = file_height = file_depth = file_colors = 0;
        has_palette = -1;
        switch(format){
            case OP_PIC_FORMAT_JPEG: {
                op_extract_jpeg_params(_buf + i, data_length, &file_width, &file_height, &file_depth, &file_colors,
                        &has_palette);
            }
                break;
            case OP_PIC_FORMAT_PNG: {
                op_extract_png_params(_buf + i, data_length, &file_width, &file_height, &file_depth, &file_colors,
                        &has_palette);
            }
                break;
            case OP_PIC_FORMAT_GIF: {
                op_extract_gif_params(_buf + i, data_length, &file_width, &file_height, &file_depth, &file_colors,
                        &has_palette);
            }
                break;
        }
        if(has_palette >= 0) {
            /*If we successfully extracted these parameters from the image, override
             any declared values.*/
            width = file_width;
            height = file_height;
            depth = file_depth;
            colors = file_colors;
        }
        /*Picture type 1 must be a 32x32 PNG.*/
        if(picture_type == 1 && (format != OP_PIC_FORMAT_PNG || width != 32 || height != 32)) {
            return OP_ENOTFORMAT;
        }
    }
    /*Adjust _buf_sz instead of using data_length to capture the terminating NUL
     for URLs.*/
    _buf_sz -= i;
    memmove(_buf, _buf + i, sizeof(*_buf) * _buf_sz);
    _buf = (unsigned char*) realloc(_buf, _buf_sz);
    if(_buf_sz > 0 && _buf == NULL) return OP_EFAULT;
    _pic->type = picture_type;
    _pic->width = width;
    _pic->height = height;
    _pic->depth = depth;
    _pic->colors = colors;
    _pic->data_length = data_length;
    _pic->data = _buf;
    _pic->format = format;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int opus_picture_tag_parse(OpusPictureTag_t *_pic, const char *_tag) {
    OpusPictureTag_t pic;
    unsigned char *buf;
    size_t base64_sz;
    size_t buf_sz;
    size_t tag_length;
    int ret;
    if(opus_tagncompare("METADATA_BLOCK_PICTURE", 22, _tag) == 0) _tag += 23;
    /*Figure out how much BASE64-encoded data we have.*/
    tag_length = strlen(_tag);
    if(tag_length & 3) return OP_ENOTFORMAT;
    base64_sz = tag_length >> 2;
    buf_sz = 3 * base64_sz;
    if(buf_sz < 32) return OP_ENOTFORMAT;
    if(_tag[tag_length - 1] == '=') buf_sz--;
    if(_tag[tag_length - 2] == '=') buf_sz--;
    if(buf_sz < 32) return OP_ENOTFORMAT;
    /*Allocate an extra byte to allow appending a terminating NUL to URL data.*/
    buf = (unsigned char*) malloc(sizeof(*buf) * (buf_sz + 1));
    if(buf == NULL) return OP_EFAULT;
    opus_picture_tag_init(&pic);
    ret = opus_picture_tag_parse_impl(&pic, _tag, buf, buf_sz, base64_sz);
    if(ret < 0) {
        opus_picture_tag_clear(&pic);
        free(buf);
    }
    else
        *_pic = *&pic;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------
void opus_picture_tag_init(OpusPictureTag_t *_pic) {
    memset(_pic, 0, sizeof(*_pic));
}
//----------------------------------------------------------------------------------------------------------------------
void opus_picture_tag_clear(OpusPictureTag_t *_pic) {
    free(_pic->description);
    free(_pic->mime_type);
    free(_pic->data);
}
//----------------------------------------------------------------------------------------------------------------------
int op_fread(void *_stream, unsigned char *_ptr, int _buf_size) {
    FILE *stream;
    size_t ret;
    /*Check for empty read.*/
    if(_buf_size <= 0) return 0;
    stream = (FILE*) _stream;
    ret = fread(_ptr, 1, _buf_size, stream);
    OP_ASSERT(ret<=(size_t)_buf_size);
    /*If ret==0 and !feof(stream), there was a read error.*/
    return ret > 0 || feof(stream) ? (int) ret : OP_EREAD;
}
//----------------------------------------------------------------------------------------------------------------------
int op_fseek(void *_stream, int64_t _offset, int _whence) {

    /*This function actually conforms to the SUSv2 and POSIX.1-2001, so we prefer
     it except on Windows.*/
    return fseeko((FILE*) _stream, (off_t) _offset, _whence);
}
//----------------------------------------------------------------------------------------------------------------------
int64_t op_ftell(void *_stream) {

    /*This function actually conforms to the SUSv2 and POSIX.1-2001, so we prefer
     it except on Windows.*/
    return ftello((FILE*) _stream);
}
//----------------------------------------------------------------------------------------------------------------------
const OpusFileCallbacks_t OP_FILE_CALLBACKS = { op_fread, op_fseek, op_ftell, (op_close_func) fclose };

const OpusFileCallbacks_t* op_get_file_callbacks(FILE *_fp) {
    (void) _fp;
    return &OP_FILE_CALLBACKS;
}
//----------------------------------------------------------------------------------------------------------------------
void* op_fopen(OpusFileCallbacks_t *_cb, const char *_path, const char *_mode) {
    FILE *fp;
    fp = fopen(_path, _mode);

    if(fp != NULL) *_cb = *op_get_file_callbacks(fp);
    return fp;
}
//----------------------------------------------------------------------------------------------------------------------
//void* op_fdopen(OpusFileCallbacks_t *_cb, int _fd, const char *_mode) {
//    FILE *fp;
//    fp = fdopen(_fd, _mode);
//    if(fp != NULL) *_cb = *op_get_file_callbacks(fp);
//    return fp;
//}
//----------------------------------------------------------------------------------------------------------------------
//void* op_freopen(OpusFileCallbacks_t *_cb, const char *_path, const char *_mode, void *_stream) {
//    FILE *fp;
//
//    fp = freopen(_path, _mode, (FILE*) _stream);
//
//    if(fp != NULL) *_cb = *op_get_file_callbacks(fp);
//    return fp;
//}
//----------------------------------------------------------------------------------------------------------------------
int op_mem_read(void *_stream, unsigned char *_ptr, int _buf_size) {
    OpusMemStream *stream;
    ptrdiff_t size;
    ptrdiff_t pos;
    stream = (OpusMemStream*) _stream;
    /*Check for empty read.*/
    if(_buf_size <= 0) return 0;
    size = stream->size;
    pos = stream->pos;
    /*Check for EOF.*/
    if(pos >= size) return 0;
    /*Check for a short read.*/
    _buf_size = (int) _min(size - pos, _buf_size);
    memcpy(_ptr, stream->data + pos, _buf_size);
    pos += _buf_size;
    stream->pos = pos;
    return _buf_size;
}
//----------------------------------------------------------------------------------------------------------------------
int op_mem_seek(void *_stream, int64_t _offset, int _whence) {
    OpusMemStream *stream;
    ptrdiff_t pos;
    stream = (OpusMemStream*) _stream;
    pos = stream->pos;
    OP_ASSERT(pos>=0);
    switch(_whence){
        case SEEK_SET: {
            /*Check for overflow:*/
            if(_offset < 0 || _offset > OP_MEM_DIFF_MAX) return -1;
            pos = (ptrdiff_t) _offset;
        }
            break;
        case SEEK_CUR: {
            /*Check for overflow:*/
            if(_offset < -pos || _offset > OP_MEM_DIFF_MAX - pos) return -1;
            pos = (ptrdiff_t) (pos + _offset);
        }
            break;
        case SEEK_END: {
            ptrdiff_t size;
            size = stream->size;
            OP_ASSERT(size>=0);
            /*Check for overflow:*/
            if(_offset < -size || _offset > OP_MEM_DIFF_MAX - size) return -1;
            pos = (ptrdiff_t) (size + _offset);
        }
            break;
        default:
            return -1;
    }
    stream->pos = pos;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int64_t op_mem_tell(void *_stream) {
    OpusMemStream *stream;
    stream = (OpusMemStream*) _stream;
    return (int64_t) stream->pos;
}
//----------------------------------------------------------------------------------------------------------------------
int op_mem_close(void *_stream) {
    free(_stream);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
const OpusFileCallbacks_t OP_MEM_CALLBACKS = { op_mem_read, op_mem_seek, op_mem_tell, op_mem_close };
//----------------------------------------------------------------------------------------------------------------------
void* op_mem_stream_create(OpusFileCallbacks_t *_cb, const unsigned char *_data, size_t _size) {
    OpusMemStream *stream;
    if(_size > OP_MEM_SIZE_MAX) return NULL;
    stream = (OpusMemStream*) malloc(sizeof(*stream));
    if(stream != NULL) {
        *_cb = *&OP_MEM_CALLBACKS;
        stream->data = _data;
        stream->size = _size;
        stream->pos = 0;
    }
    return stream;
}
//----------------------------------------------------------------------------------------------------------------------
