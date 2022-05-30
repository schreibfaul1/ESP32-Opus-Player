/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE Ogg CONTAINER SOURCE CODE.              *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE OggVorbis SOURCE CODE IS (C) COPYRIGHT 1994-2014             *
 * by the Xiph.Org Foundation http://www.xiph.org/                  *
 *                                                                  *
 ********************************************************************

  function: packing variable sized words into an octet stream

 ********************************************************************/

/* We're 'LSb' endian; if we write a word but read individual bits,
   then we'll read the lsb first */

#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include "ogg.h"

#define BUFFER_INCREMENT 256

static const unsigned long mask[]=
{0x00000000,0x00000001,0x00000003,0x00000007,0x0000000f,
 0x0000001f,0x0000003f,0x0000007f,0x000000ff,0x000001ff,
 0x000003ff,0x000007ff,0x00000fff,0x00001fff,0x00003fff,
 0x00007fff,0x0000ffff,0x0001ffff,0x0003ffff,0x0007ffff,
 0x000fffff,0x001fffff,0x003fffff,0x007fffff,0x00ffffff,
 0x01ffffff,0x03ffffff,0x07ffffff,0x0fffffff,0x1fffffff,
 0x3fffffff,0x7fffffff,0xffffffff };

static const unsigned int mask8B[]=
{0x00,0x80,0xc0,0xe0,0xf0,0xf8,0xfc,0xfe,0xff};

/* bits <= 32 */
long oggpack_read(oggpack_buffer *b,int bits){
  long ret;
  unsigned long m;

  if(bits<0 || bits>32) goto err;
  m=mask[bits];
  bits+=b->endbit;

  if(b->endbyte >= b->storage-4){
    /* not the main path */
    if(b->endbyte > b->storage-((bits+7)>>3)) goto overflow;
    /* special case to avoid reading b->ptr[0], which might be past the end of
        the buffer; also skips some useless accounting */
    else if(!bits)return(0L);
  }

  ret=b->ptr[0]>>b->endbit;
  if(bits>8){
    ret|=b->ptr[1]<<(8-b->endbit);
    if(bits>16){
      ret|=b->ptr[2]<<(16-b->endbit);
      if(bits>24){
        ret|=b->ptr[3]<<(24-b->endbit);
        if(bits>32 && b->endbit){
          ret|=b->ptr[4]<<(32-b->endbit);
        }
      }
    }
  }
  ret&=m;
  b->ptr+=bits/8;
  b->endbyte+=bits/8;
  b->endbit=bits&7;
  return ret;

 overflow:
 err:
  b->ptr=NULL;
  b->endbyte=b->storage;
  b->endbit=1;
  return -1L;
}

/* bits <= 32 */
long oggpackB_read(oggpack_buffer *b,int bits){
  long ret;
  long m=32-bits;

  if(m<0 || m>32) goto err;
  bits+=b->endbit;

  if(b->endbyte+4>=b->storage){
    /* not the main path */
    if(b->endbyte > b->storage-((bits+7)>>3)) goto overflow;
    /* special case to avoid reading b->ptr[0], which might be past the end of
        the buffer; also skips some useless accounting */
    else if(!bits)return(0L);
  }

  ret=b->ptr[0]<<(24+b->endbit);
  if(bits>8){
    ret|=b->ptr[1]<<(16+b->endbit);
    if(bits>16){
      ret|=b->ptr[2]<<(8+b->endbit);
      if(bits>24){
        ret|=b->ptr[3]<<(b->endbit);
        if(bits>32 && b->endbit)
          ret|=b->ptr[4]>>(8-b->endbit);
      }
    }
  }
  ret=((ret&0xffffffffUL)>>(m>>1))>>((m+1)>>1);

  b->ptr+=bits/8;
  b->endbyte+=bits/8;
  b->endbit=bits&7;
  return ret;

 overflow:
 err:
  b->ptr=NULL;
  b->endbyte=b->storage;
  b->endbit=1;
  return -1L;
}

long oggpack_read1(oggpack_buffer *b){
  long ret;

  if(b->endbyte >= b->storage) goto overflow;
  ret=(b->ptr[0]>>b->endbit)&1;

  b->endbit++;
  if(b->endbit>7){
    b->endbit=0;
    b->ptr++;
    b->endbyte++;
  }
  return ret;

 overflow:
  b->ptr=NULL;
  b->endbyte=b->storage;
  b->endbit=1;
  return -1L;
}

long oggpackB_read1(oggpack_buffer *b){
  long ret;

  if(b->endbyte >= b->storage) goto overflow;
  ret=(b->ptr[0]>>(7-b->endbit))&1;

  b->endbit++;
  if(b->endbit>7){
    b->endbit=0;
    b->ptr++;
    b->endbyte++;
  }
  return ret;

 overflow:
  b->ptr=NULL;
  b->endbyte=b->storage;
  b->endbit=1;
  return -1L;
}

long oggpack_bytes(oggpack_buffer *b){
  return(b->endbyte+(b->endbit+7)/8);
}

long oggpack_bits(oggpack_buffer *b){
  return(b->endbyte*8+b->endbit);
}

long oggpackB_bytes(oggpack_buffer *b){
  return oggpack_bytes(b);
}

long oggpackB_bits(oggpack_buffer *b){
  return oggpack_bits(b);
}

unsigned char *oggpack_get_buffer(oggpack_buffer *b){
  return(b->buffer);
}

unsigned char *oggpackB_get_buffer(oggpack_buffer *b){
  return oggpack_get_buffer(b);
}


#undef BUFFER_INCREMENT
