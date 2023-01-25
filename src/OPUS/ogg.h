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

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef struct {
    void  *iov_base;
    size_t iov_len;
} ogg_iovec_t;

typedef struct {
    int32_t endbyte;
    int32_t endbit;

    uint8_t *buffer;
    uint8_t *ptr;
    int32_t  storage;
} oggpack_buffer;

/* ogg_page is used to encapsulate the data in one Ogg bitstream page *****/

typedef struct {
    uint8_t *header;
    int32_t  header_len;
    uint8_t *body;
    int32_t  body_len;
} ogg_page;

/* ogg_stream_state contains the current encode/decode state of a logical Ogg bitstream */
typedef struct {
    uint8_t *body_data; 	/* bytes from packet bodies */
    int32_t body_storage;        	/* storage elements allocated */
    int32_t body_fill;           	/* elements stored; fill mark */
    int32_t body_returned;       	/* elements of fill returned */
    int32_t *lacing_vals;         	/* The values that will go to the segment table */
    int64_t *granule_vals;    	/* granulepos values for headers. */
    int32_t lacing_storage;
    int32_t lacing_fill;
    int32_t lacing_packet;
    int32_t lacing_returned;
    uint8_t header[282]; 	/* working space for header encode */
    int32_t header_fill;
    int32_t e_o_s; 					/* set when we have buffered the last packet in the logical bitstream */
    int32_t b_o_s; 					/* set after we've written the initial page of a logical bitstream */
    int32_t serialno;
    int32_t pageno;
    int64_t packetno; /* sequence number for decode; */
	int64_t granulepos;
} ogg_stream_state;

/* ogg_packet is used to encapsulate the data and metadata belonging to a single raw Ogg/Vorbis packet */
typedef struct {
    uint8_t *packet;
    int32_t bytes;
    int32_t b_o_s;
    int32_t e_o_s;
    int64_t granulepos;
    int64_t packetno; /* sequence number for decode; */
} ogg_packet;

typedef struct {
    uint8_t *data;
    int32_t storage;
    int32_t fill;
    int32_t returned;

    int32_t unsynced;
    int32_t headerbytes;
    int32_t bodybytes;
} ogg_sync_state;


    /* Ogg BITSTREAM PRIMITIVES: decoding **************************/

    int32_t ogg_sync_clear(ogg_sync_state *oy);
    int32_t ogg_sync_check(ogg_sync_state *oy);

    char *ogg_sync_buffer(ogg_sync_state *oy, int32_t size);
    int32_t ogg_sync_wrote(ogg_sync_state *oy, int32_t bytes);
    int32_t ogg_sync_pageseek(ogg_sync_state *oy, ogg_page *og);
    int32_t ogg_stream_pagein(ogg_stream_state *os, ogg_page *og);
    int32_t ogg_stream_packetout(ogg_stream_state *os, ogg_packet *op);

    /* Ogg BITSTREAM PRIMITIVES: general ***************************/

    int32_t ogg_stream_init(ogg_stream_state *os, int32_t serialno);
    int32_t ogg_stream_clear(ogg_stream_state *os);
    int32_t ogg_stream_reset(ogg_stream_state *os);
    int32_t ogg_stream_reset_serialno(ogg_stream_state *os, int32_t serialno);
    int32_t ogg_page_bos(const ogg_page *og);
    int64_t ogg_page_granulepos(const ogg_page *og);
    int32_t getSerialNo(const ogg_page *og);
    int32_t ogg_page_packets(const ogg_page *og);
#ifdef __cplusplus
}
#endif

#endif  /* _OGG_H */
