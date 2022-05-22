/* Copyright (c) 2017 Google Inc.
   Written by Andrew Allen */
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

//#ifdef HAVE_CONFIG_H
#include "config.h"
//#endif

#include "celt/arch.h"
#include "celt/float_cast.h"
#include "opus_private.h"
#include "opus_defines.h"
#include "mapping_matrix.h"

#define MATRIX_INDEX(nb_rows, row, col) (nb_rows * col + row)

int32_t mapping_matrix_get_size(int rows, int cols)
{
  int32_t size;

  /* Mapping Matrix must only support up to 255 channels in or out.
   * Additionally, the total cell count must be <= 65004 octets in order
   * for the matrix to be stored in an OGG header.
   */
  if (rows > 255 || cols > 255)
      return 0;
  size = rows * (int32_t)cols * sizeof(int16_t);
  if (size > 65004)
    return 0;

  return align(sizeof(MappingMatrix)) + align(size);
}

int16_t *mapping_matrix_get_data(const MappingMatrix *matrix)
{
  /* void* cast avoids clang -Wcast-align warning */
  return (int16_t*)(void*)((char*)matrix + align(sizeof(MappingMatrix)));
}

void mapping_matrix_init(MappingMatrix * const matrix,
  int rows, int cols, int gain, const int16_t *data, int32_t data_size)
{
  int i;
  int16_t *ptr;

#if !defined(ENABLE_ASSERTIONS)
  (void)data_size;
#endif
  celt_assert(align(data_size) == align(rows * cols * sizeof(int16_t)));

  matrix->rows = rows;
  matrix->cols = cols;
  matrix->gain = gain;
  ptr = mapping_matrix_get_data(matrix);
  for (i = 0; i < rows * cols; i++)
  {
     ptr[i] = data[i];
  }
}


void mapping_matrix_multiply_channel_in_short(
    const MappingMatrix *matrix,
    const int16_t *input,
    int input_rows,
    opus_val16 *output,
    int output_row,
    int output_rows,
    int frame_size)
{
  /* Matrix data is ordered col-wise. */
  int16_t* matrix_data;
  int i, col;

  celt_assert(input_rows <= matrix->cols && output_rows <= matrix->rows);

  matrix_data = mapping_matrix_get_data(matrix);

  for (i = 0; i < frame_size; i++)
  {
    opus_val32 tmp = 0;
    for (col = 0; col < input_rows; col++)
    {
      tmp +=
        ((int32_t)matrix_data[MATRIX_INDEX(matrix->rows, output_row, col)] *
        (int32_t)input[MATRIX_INDEX(input_rows, col, i)]) >> 8;
    }
    output[output_rows * i] = (int16_t)((tmp + 64) >> 7);
  }
}

void mapping_matrix_multiply_channel_out_short(
    const MappingMatrix *matrix,
    const opus_val16 *input,
    int input_row,
    int input_rows,
    int16_t *output,
    int output_rows,
    int frame_size)
{
  /* Matrix data is ordered col-wise. */
  int16_t* matrix_data;
  int i, row;
  int32_t input_sample;

  celt_assert(input_rows <= matrix->cols && output_rows <= matrix->rows);

  matrix_data = mapping_matrix_get_data(matrix);

  for (i = 0; i < frame_size; i++)
  {
    input_sample = (int32_t)input[input_rows * i];
    for (row = 0; row < output_rows; row++)
    {
      int32_t tmp =
        (int32_t)matrix_data[MATRIX_INDEX(matrix->rows, row, input_row)] *
        input_sample;
      output[MATRIX_INDEX(output_rows, row, i)] += (tmp + 16384) >> 15;
    }
  }
}

const MappingMatrix mapping_matrix_foa_mixing = { 6, 6, 0 };
const int16_t mapping_matrix_foa_mixing_data[36] = {
     16384,      0, -16384,  23170,      0,      0,  16384,  23170,
     16384,      0,      0,      0,  16384,      0, -16384, -23170,
         0,      0,  16384, -23170,  16384,      0,      0,      0,
         0,      0,      0,      0,  32767,      0,      0,      0,
         0,      0,      0,  32767
};

const MappingMatrix mapping_matrix_soa_mixing = { 11, 11, 0 };
const int16_t mapping_matrix_soa_mixing_data[121] = {
     10923,   7723,  13377, -13377,  11585,   9459,   7723, -16384,
     -6689,      0,      0,  10923,   7723,  13377,  13377, -11585,
      9459,   7723,  16384,  -6689,      0,      0,  10923, -15447,
     13377,      0,      0, -18919,   7723,      0,  13377,      0,
         0,  10923,   7723, -13377, -13377,  11585,  -9459,   7723,
     16384,  -6689,      0,      0,  10923,  -7723,      0,  13377,
    -16384,      0, -15447,      0,   9459,      0,      0,  10923,
     -7723,      0, -13377,  16384,      0, -15447,      0,   9459,
         0,      0,  10923,  15447,      0,      0,      0,      0,
    -15447,      0, -18919,      0,      0,  10923,   7723, -13377,
     13377, -11585,  -9459,   7723, -16384,  -6689,      0,      0,
     10923, -15447, -13377,      0,      0,  18919,   7723,      0,
     13377,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,      0,  32767,      0,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
     32767
};

const MappingMatrix mapping_matrix_toa_mixing = { 18, 18, 0 };
const int16_t mapping_matrix_toa_mixing_data[324] = {
      8208,      0,   -881,  14369,      0,      0,  -8192,  -4163,
     13218,      0,      0,      0,  11095,  -8836,  -6218,  14833,
         0,      0,   8208, -10161,    881,  10161, -13218,  -2944,
     -8192,   2944,      0, -10488,  -6218,   6248, -11095,  -6248,
         0, -10488,      0,      0,   8208,  10161,    881, -10161,
    -13218,   2944,  -8192,  -2944,      0,  10488,  -6218,  -6248,
    -11095,   6248,      0,  10488,      0,      0,   8176,   5566,
    -11552,   5566,   9681, -11205,   8192, -11205,      0,   4920,
    -15158,   9756,  -3334,   9756,      0,  -4920,      0,      0,
      8176,   7871,  11552,      0,      0,  15846,   8192,      0,
     -9681,  -6958,      0,  13797,   3334,      0, -15158,      0,
         0,      0,   8176,      0,  11552,   7871,      0,      0,
      8192,  15846,   9681,      0,      0,      0,   3334,  13797,
     15158,   6958,      0,      0,   8176,   5566, -11552,  -5566,
     -9681, -11205,   8192,  11205,      0,   4920,  15158,   9756,
     -3334,  -9756,      0,   4920,      0,      0,   8208,  14369,
      -881,      0,      0,  -4163,  -8192,      0, -13218, -14833,
         0,  -8836,  11095,      0,   6218,      0,      0,      0,
      8208,  10161,    881,  10161,  13218,   2944,  -8192,   2944,
         0,  10488,   6218,  -6248, -11095,  -6248,      0, -10488,
         0,      0,   8208, -14369,   -881,      0,      0,   4163,
     -8192,      0, -13218,  14833,      0,   8836,  11095,      0,
      6218,      0,      0,      0,   8208,      0,   -881, -14369,
         0,      0,  -8192,   4163,  13218,      0,      0,      0,
     11095,   8836,  -6218, -14833,      0,      0,   8176,  -5566,
    -11552,   5566,  -9681,  11205,   8192, -11205,      0,  -4920,
     15158,  -9756,  -3334,   9756,      0,  -4920,      0,      0,
      8176,      0,  11552,  -7871,      0,      0,   8192, -15846,
      9681,      0,      0,      0,   3334, -13797,  15158,  -6958,
         0,      0,   8176,  -7871,  11552,      0,      0, -15846,
      8192,      0,  -9681,   6958,      0, -13797,   3334,      0,
    -15158,      0,      0,      0,   8176,  -5566, -11552,  -5566,
      9681,  11205,   8192,  11205,      0,  -4920, -15158,  -9756,
     -3334,  -9756,      0,   4920,      0,      0,   8208, -10161,
       881, -10161,  13218,  -2944,  -8192,  -2944,      0, -10488,
      6218,   6248, -11095,   6248,      0,  10488,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
     32767,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,  32767
};

const MappingMatrix mapping_matrix_foa_demixing = { 6, 6, 0 };
const int16_t mapping_matrix_foa_demixing_data[36] = {
     16384,  16384,  16384,  16384,      0,      0,      0,  23170,
         0, -23170,      0,      0, -16384,  16384, -16384,  16384,
         0,      0,  23170,      0, -23170,      0,      0,      0,
         0,      0,      0,      0,  32767,      0,      0,      0,
         0,      0,      0,  32767
};

const MappingMatrix mapping_matrix_soa_demixing = { 11, 11, 3050 };
const int16_t mapping_matrix_soa_demixing_data[121] = {
      2771,   2771,   2771,   2771,   2771,   2771,   2771,   2771,
      2771,      0,      0,  10033,  10033, -20066,  10033,  14189,
     14189, -28378,  10033, -20066,      0,      0,   3393,   3393,
      3393,  -3393,      0,      0,      0,  -3393,  -3393,      0,
         0, -17378,  17378,      0, -17378, -24576,  24576,      0,
     17378,      0,      0,      0, -14189,  14189,      0, -14189,
    -28378,  28378,      0,  14189,      0,      0,      0,   2399,
      2399,  -4799,  -2399,      0,      0,      0,  -2399,   4799,
         0,      0,   1959,   1959,   1959,   1959,  -3918,  -3918,
     -3918,   1959,   1959,      0,      0,  -4156,   4156,      0,
      4156,      0,      0,      0,  -4156,      0,      0,      0,
      8192,   8192, -16384,   8192,  16384,  16384, -32768,   8192,
    -16384,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,      0,   8312,      0,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
      8312
};

const MappingMatrix mapping_matrix_toa_demixing = { 18, 18, 0 };
const int16_t mapping_matrix_toa_demixing_data[324] = {
      8192,   8192,   8192,   8192,   8192,   8192,   8192,   8192,
      8192,   8192,   8192,   8192,   8192,   8192,   8192,   8192,
         0,      0,      0,  -9779,   9779,   6263,   8857,      0,
      6263,  13829,   9779, -13829,      0,  -6263,      0,  -8857,
     -6263,  -9779,      0,      0,  -3413,   3413,   3413, -11359,
     11359,  11359, -11359,  -3413,   3413,  -3413,  -3413, -11359,
     11359,  11359, -11359,   3413,      0,      0,  13829,   9779,
     -9779,   6263,      0,   8857,  -6263,      0,   9779,      0,
    -13829,   6263,  -8857,      0,  -6263,  -9779,      0,      0,
         0, -15617, -15617,   6406,      0,      0,  -6406,      0,
     15617,      0,      0,  -6406,      0,      0,   6406,  15617,
         0,      0,      0,  -5003,   5003, -10664,  15081,      0,
    -10664,  -7075,   5003,   7075,      0,  10664,      0, -15081,
     10664,  -5003,      0,      0,  -8176,  -8176,  -8176,   8208,
      8208,   8208,   8208,  -8176,  -8176,  -8176,  -8176,   8208,
      8208,   8208,   8208,  -8176,      0,      0,  -7075,   5003,
     -5003, -10664,      0,  15081,  10664,      0,   5003,      0,
      7075, -10664, -15081,      0,  10664,  -5003,      0,      0,
     15617,      0,      0,      0,  -6406,   6406,      0, -15617,
         0, -15617,  15617,      0,   6406,  -6406,      0,      0,
         0,      0,      0, -11393,  11393,   2993,  -4233,      0,
      2993, -16112,  11393,  16112,      0,  -2993,      0,   4233,
     -2993, -11393,      0,      0,      0,  -9974,  -9974, -13617,
         0,      0,  13617,      0,   9974,      0,      0,  13617,
         0,      0, -13617,   9974,      0,      0,      0,   5579,
     -5579,  10185,  14403,      0,  10185,  -7890,  -5579,   7890,
         0, -10185,      0, -14403, -10185,   5579,      0,      0,
     11826, -11826, -11826,   -901,    901,    901,   -901,  11826,
    -11826,  11826,  11826,   -901,    901,    901,   -901, -11826,
         0,      0,  -7890,  -5579,   5579,  10185,      0,  14403,
    -10185,      0,  -5579,      0,   7890,  10185, -14403,      0,
    -10185,   5579,      0,      0,  -9974,      0,      0,      0,
    -13617,  13617,      0,   9974,      0,   9974,  -9974,      0,
     13617, -13617,      0,      0,      0,      0,  16112, -11393,
     11393,  -2993,      0,   4233,   2993,      0, -11393,      0,
    -16112,  -2993,  -4233,      0,   2993,  11393,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
     32767,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,      0,      0,      0,      0,      0,
         0,      0,      0,  32767
};

