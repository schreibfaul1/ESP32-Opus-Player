/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE OggVorbis SOFTWARE CODEC SOURCE CODE.   *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE OggVorbis SOURCE CODE IS (C) COPYRIGHT 1994-2007             *
 * by the Xiph.Org Foundation http://www.xiph.org/                  *
 *                                                                  *
 ********************************************************************

 function: toplevel libogg include

 ********************************************************************/
#ifndef _OGG_H
#define _OGG_H

#include "Arduino.h"


typedef struct {
  void *iov_base;
  size_t iov_len;
} ogg_iovec_t;

typedef struct {
  long endbyte;
  int  endbit;

  unsigned char *buffer;
  unsigned char *ptr;
  long storage;
} oggpack_buffer;

/* ogg_page is used to encapsulate the data in one Ogg bitstream page *****/

typedef struct {
  unsigned char *header;
  long header_len;
  unsigned char *body;
  long body_len;
} ogg_page;

/* ogg_stream_state contains the current encode/decode state of a logical Ogg bitstream */
typedef struct {
    unsigned char *body_data; 	/* bytes from packet bodies */
    long body_storage;        	/* storage elements allocated */
    long body_fill;           	/* elements stored; fill mark */
    long body_returned;       	/* elements of fill returned */
    int *lacing_vals;         	/* The values that will go to the segment table */
    int64_t *granule_vals;    	/* granulepos values for headers. */
    long lacing_storage;
    long lacing_fill;
    long lacing_packet;
    long lacing_returned;
    unsigned char header[282]; 	/* working space for header encode */
    int header_fill;
    int e_o_s; 					/* set when we have buffered the last packet in the logical bitstream */
    int b_o_s; 					/* set after we've written the initial page of a logical bitstream */
    long serialno;
    long pageno;
    int64_t packetno; /* sequence number for decode; */
	int64_t granulepos;
} ogg_stream_state;

/* ogg_packet is used to encapsulate the data and metadata belonging to a single raw Ogg/Vorbis packet */
typedef struct {
    unsigned char *packet;
    long bytes;
    long b_o_s;
    long e_o_s;
    int64_t granulepos;
    int64_t packetno; /* sequence number for decode; */
} ogg_packet;

typedef struct {
    unsigned char *data;
    int storage;
    int fill;
    int returned;

    int unsynced;
    int headerbytes;
    int bodybytes;
} ogg_sync_state;


    /* Ogg BITSTREAM PRIMITIVES: decoding **************************/

    int ogg_sync_init(ogg_sync_state *oy);
    int ogg_sync_clear(ogg_sync_state *oy);
    int ogg_sync_reset(ogg_sync_state *oy);
    int ogg_sync_destroy(ogg_sync_state *oy);
    int ogg_sync_check(ogg_sync_state *oy);

    char *ogg_sync_buffer(ogg_sync_state *oy, long size);
    int ogg_sync_wrote(ogg_sync_state *oy, long bytes);
    long ogg_sync_pageseek(ogg_sync_state *oy, ogg_page *og);
    // extern int ogg_sync_pageout(ogg_sync_state *oy, ogg_page *og);
    int ogg_stream_pagein(ogg_stream_state *os, ogg_page *og);
    int ogg_stream_packetout(ogg_stream_state *os, ogg_packet *op);
    // extern int ogg_stream_packetpeek(ogg_stream_state *os, ogg_packet *op);

    /* Ogg BITSTREAM PRIMITIVES: general ***************************/

    int ogg_stream_init(ogg_stream_state *os, int serialno);
    int ogg_stream_clear(ogg_stream_state *os);
    int ogg_stream_reset(ogg_stream_state *os);
    int ogg_stream_reset_serialno(ogg_stream_state *os, int serialno);
    // extern int ogg_stream_destroy(ogg_stream_state *os);
    // extern int ogg_stream_check(ogg_stream_state *os);
    // extern int ogg_stream_eos(ogg_stream_state *os);

    // extern void ogg_page_checksum_set(ogg_page *og);

    // extern int ogg_page_version(const ogg_page *og);
    // extern int ogg_page_continued(const ogg_page *og);
    int ogg_page_bos(const ogg_page *og);
    // extern int ogg_page_eos(const ogg_page *og);
    int64_t ogg_page_granulepos(const ogg_page *og);
    int getSerialNo(const ogg_page *og);
    // extern long ogg_page_pageno(const ogg_page *og);
    int ogg_page_packets(const ogg_page *og);

    // extern void ogg_packet_clear(ogg_packet *op);



#endif  /* _OGG_H */
