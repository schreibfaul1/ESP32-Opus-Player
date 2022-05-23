/* Copyright (c) 2007-2008 CSIRO
   Copyright (c) 2007-2010 Xiph.Org Foundation
   Copyright (c) 2008 Gregory Maxwell
   Written by Jean-Marc Valin and Gregory Maxwell */
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

#define CELT_C


#include "celt.h"
#include "os_support.h"
#include "mdct.h"
#include <math.h>
#include "pitch.h"
#include "bands.h"
#include "modes.h"
#include "entcode.h"
#include "quant_bands.h"
#include "rate.h"
#include "stack_alloc.h"
#include "mathops.h"
#include "float_cast.h"
#include <stdarg.h>
#include "celt_lpc.h"
#include "vq.h"
#include <pgmspace.h>


/*For each V(N,K) supported, we will access element U(min(N,K+1),max(N,K+1)). Thus, the number of entries in row I is
  the larger of the maximum number of pulses we will ever allocate for a given N=I (K=128, or however many fit in
  32 bits, whichever is smaller), plus one, and the maximum N for which K=I-1 pulses fit in 32 bits.
  The largest band size in an Opus Custom mode is 208. Otherwise, we can limit things to the set of N which can be
  achieved by splitting a band from a
  standard Opus mode: 176, 144, 96, 88, 72, 64, 48,44, 36, 32, 24, 22, 18, 16, 8, 4, 2).*/

static const uint32_t CELT_PVQ_U_DATA[1272] PROGMEM = {

    /*N=0, K=0...176:*/
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    /*N=1, K=1...176:*/
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,

    /*N=2, K=2...176:*/
    3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61,
    63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107, 109, 111, 113, 115,
    117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143, 145, 147, 149, 151, 153, 155, 157, 159, 161,
    163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187, 189, 191, 193, 195, 197, 199, 201, 203, 205, 207,
    209, 211, 213, 215, 217, 219, 221, 223, 225, 227, 229, 231, 233, 235, 237, 239, 241, 243, 245, 247, 249, 251, 253,
    255, 257, 259, 261, 263, 265, 267, 269, 271, 273, 275, 277, 279, 281, 283, 285, 287, 289, 291, 293, 295, 297, 299,
    301, 303, 305, 307, 309, 311, 313, 315, 317, 319, 321, 323, 325, 327, 329, 331, 333, 335, 337, 339, 341, 343, 345,
    347, 349, 351,

    /*N=3, K=3...176:*/
    13, 25, 41, 61, 85, 113, 145, 181, 221, 265, 313, 365, 421, 481, 545, 613, 685, 761, 841, 925, 1013, 1105, 1201,
    1301, 1405, 1513, 1625, 1741, 1861, 1985, 2113, 2245, 2381, 2521, 2665, 2813, 2965, 3121, 3281, 3445, 3613, 3785,
    3961, 4141, 4325, 4513, 4705, 4901, 5101, 5305, 5513, 5725, 5941, 6161, 6385, 6613, 6845, 7081, 7321, 7565, 7813,
    8065, 8321, 8581, 8845, 9113, 9385, 9661, 9941, 10225, 10513, 10805, 11101, 11401, 11705, 12013, 12325, 12641,
    12961, 13285, 13613, 13945, 14281, 14621, 14965, 15313, 15665, 16021, 16381, 16745, 17113, 17485, 17861, 18241,
    18625, 19013, 19405, 19801, 20201, 20605, 21013, 21425, 21841, 22261, 22685, 23113, 23545, 23981, 24421, 24865,
    25313, 25765, 26221, 26681, 27145, 27613, 28085, 28561, 29041, 29525, 30013, 30505, 31001, 31501, 32005, 32513,
    33025, 33541, 34061, 34585, 35113, 35645, 36181, 36721, 37265, 37813, 38365, 38921, 39481, 40045, 40613, 41185,
    41761, 42341, 42925, 43513, 44105, 44701, 45301, 45905, 46513, 47125, 47741, 48361, 48985, 49613, 50245, 50881,
    51521, 52165, 52813, 53465, 54121, 54781, 55445, 56113, 56785, 57461, 58141, 58825, 59513, 60205, 60901, 61601,

    /*N=4, K=4...176:*/
    63, 129, 231, 377, 575, 833, 1159, 1561, 2047, 2625, 3303, 4089, 4991, 6017, 7175, 8473, 9919, 11521, 13287, 15225,
    17343, 19649, 22151, 24857, 27775, 30913, 34279, 37881, 41727, 45825, 50183, 54809, 59711, 64897, 70375, 76153,
    82239, 88641, 95367, 102425, 109823, 117569, 125671, 134137, 142975, 152193, 161799, 171801, 182207, 193025, 204263,
    215929, 228031, 240577, 253575, 267033, 280959, 295361, 310247, 325625, 341503, 357889, 374791, 392217, 410175,
    428673, 447719, 467321, 487487, 508225, 529543, 551449, 573951, 597057, 620775, 645113, 670079, 695681, 721927,
    748825, 776383, 804609, 833511, 863097, 893375, 924353, 956039, 988441, 1021567, 1055425, 1090023, 1125369, 1161471,
    1198337, 1235975, 1274393, 1313599, 1353601, 1394407, 1436025, 1478463, 1521729, 1565831, 1610777, 1656575, 1703233,
    1750759, 1799161, 1848447, 1898625, 1949703, 2001689, 2054591, 2108417, 2163175, 2218873, 2275519, 2333121, 2391687,
    2451225, 2511743, 2573249, 2635751, 2699257, 2763775, 2829313, 2895879, 2963481, 3032127, 3101825, 3172583, 3244409,
    3317311, 3391297, 3466375, 3542553, 3619839, 3698241, 3777767, 3858425, 3940223, 4023169, 4107271, 4192537, 4278975,
    4366593, 4455399, 4545401, 4636607, 4729025, 4822663, 4917529, 5013631, 5110977, 5209575, 5309433, 5410559, 5512961,
    5616647, 5721625, 5827903, 5935489, 6044391, 6154617, 6266175, 6379073, 6493319, 6608921, 6725887, 6844225, 6963943,
    7085049, 7207551,

    /*N=5, K=5...176:*/
    321, 681, 1289, 2241, 3649, 5641, 8361, 11969, 16641, 22569, 29961, 39041, 50049, 63241, 78889, 97281, 118721,
    143529, 172041, 204609, 241601, 283401, 330409, 383041, 441729, 506921, 579081, 658689, 746241, 842249, 947241,
    1061761, 1186369, 1321641, 1468169, 1626561, 1797441, 1981449, 2179241, 2391489, 2618881, 2862121, 3121929, 3399041,
    3694209, 4008201, 4341801, 4695809, 5071041, 5468329, 5888521, 6332481, 6801089, 7295241, 7815849, 8363841, 8940161,
    9545769, 10181641, 10848769, 11548161, 12280841, 13047849, 13850241, 14689089, 15565481, 16480521, 17435329,
    18431041, 19468809, 20549801, 21675201, 22846209, 24064041, 25329929, 26645121, 28010881, 29428489, 30899241,
    32424449, 34005441, 35643561, 37340169, 39096641, 40914369, 42794761, 44739241, 46749249, 48826241, 50971689,
    53187081, 55473921, 57833729, 60268041, 62778409, 65366401, 68033601, 70781609, 73612041, 76526529, 79526721,
    82614281, 85790889, 89058241, 92418049, 95872041, 99421961, 103069569, 106816641, 110664969, 114616361, 118672641,
    122835649, 127107241, 131489289, 135983681, 140592321, 145317129, 150160041, 155123009, 160208001, 165417001,
    170752009, 176215041, 181808129, 187533321, 193392681, 199388289, 205522241, 211796649, 218213641, 224775361,
    231483969, 238341641, 245350569, 252512961, 259831041, 267307049, 274943241, 282741889, 290705281, 298835721,
    307135529, 315607041, 324252609, 333074601, 342075401, 351257409, 360623041, 370174729, 379914921, 389846081,
    399970689, 410291241, 420810249, 431530241, 442453761, 453583369, 464921641, 476471169, 488234561, 500214441,
    512413449, 524834241, 537479489, 550351881, 563454121, 576788929, 590359041, 604167209, 618216201, 632508801,

    /*N=6, K=6...96:*/
    1683, 3653, 7183, 13073, 22363, 36365, 56695, 85305, 124515, 177045, 246047, 335137, 448427, 590557, 766727, 982729,
    1244979, 1560549, 1937199, 2383409, 2908411, 3522221, 4235671, 5060441, 6009091, 7095093, 8332863, 9737793,
    11326283, 13115773, 15124775, 17372905, 19880915, 22670725, 25765455, 29189457, 32968347, 37129037, 41699767,
    46710137, 52191139, 58175189, 64696159, 71789409, 79491819, 87841821, 96879431, 106646281, 117185651, 128542501,
    140763503, 153897073, 167993403, 183104493, 199284183, 216588185, 235074115, 254801525, 275831935, 298228865,
    322057867, 347386557, 374284647, 402823977, 433078547, 465124549, 499040399, 534906769, 572806619, 612825229,
    655050231, 699571641, 746481891, 795875861, 847850911, 902506913, 959946283, 1020274013, 1083597703, 1150027593,
    1219676595, 1292660325, 1369097135, 1449108145, 1532817275, 1620351277, 1711839767, 1807415257, 1907213187,
    2011371957, 2120032959,

    /*N=7, K=7...54*/
    8989, 19825, 40081, 75517, 134245, 227305, 369305, 579125, 880685, 1303777, 1884961, 2668525, 3707509, 5064793,
    6814249, 9041957, 11847485, 15345233, 19665841, 24957661, 31388293, 39146185, 48442297, 59511829, 72616013,
    88043969, 106114625, 127178701, 151620757, 179861305, 212358985, 249612805, 292164445, 340600625, 395555537,
    457713341, 527810725, 606639529, 695049433, 793950709, 904317037, 1027188385, 1163673953, 1314955181, 1482288821,
    1667010073, 1870535785, 2094367717,

    /*N=8, K=8...37*/
    48639, 108545, 224143, 433905, 795455, 1392065, 2340495, 3800305, 5984767, 9173505, 13726991, 20103025, 28875327,
    40754369, 56610575, 77500017, 104692735, 139703809, 184327311, 240673265, 311207743, 398796225, 506750351,
    638878193, 799538175, 993696769, 1226990095, 1505789553, 1837271615, 2229491905U,

    /*N=9, K=9...28:*/
    265729, 598417, 1256465, 2485825, 4673345, 8405905, 14546705, 24331777, 39490049, 62390545, 96220561, 145198913,
    214828609, 312193553, 446304145, 628496897, 872893441, 1196924561, 1621925137, 2173806145U,

    /*N=10, K=10...24:*/
    1462563, 3317445, 7059735, 14218905, 27298155, 50250765, 89129247, 152951073, 254831667, 413442773, 654862247,
    1014889769, 1541911931, 2300409629U, 3375210671U,
    /*N=11, K=11...19:*/
    8097453, 18474633, 39753273, 81270333, 158819253, 298199265, 540279585, 948062325, 1616336765,

    /*N=12, K=12...18:*/
    45046719, 103274625, 224298231, 464387817, 921406335, 1759885185, 3248227095U,
    /*N=13, K=13...16:*/
    251595969, 579168825, 1267854873, 2653649025U,
    /*N=14, K=14:*/
    1409933619};

static const uint32_t *const CELT_PVQ_U_ROW[15] PROGMEM = {
    CELT_PVQ_U_DATA + 0,    CELT_PVQ_U_DATA + 176,  CELT_PVQ_U_DATA + 351,  CELT_PVQ_U_DATA + 525,
    CELT_PVQ_U_DATA + 698,  CELT_PVQ_U_DATA + 870,  CELT_PVQ_U_DATA + 1041, CELT_PVQ_U_DATA + 1131,
    CELT_PVQ_U_DATA + 1178, CELT_PVQ_U_DATA + 1207, CELT_PVQ_U_DATA + 1226, CELT_PVQ_U_DATA + 1240,
    CELT_PVQ_U_DATA + 1248, CELT_PVQ_U_DATA + 1254, CELT_PVQ_U_DATA + 1257};


//----------------------------------------------------------------------------------------------------------------------

int resampling_factor(int32_t rate){
    int ret;
    switch (rate){
        case 48000: ret = 1;  break;
        case 24000: ret = 2;  break;
        case 16000: ret = 3;  break;
        case 12000: ret = 4;  break;
        case 8000:  ret = 6;  break;
        default:    celt_assert(0);  ret = 0;  break;
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

void comb_filter_const_c(int32_t *y, int32_t *x, int T, int N, int16_t g10, int16_t g11, int16_t g12) {
    int32_t x0, x1, x2, x3, x4;
    int i;
    x4 = x[-T - 2];
    x3 = x[-T - 1];
    x2 = x[-T];
    x1 = x[-T + 1];
    for (i = 0; i < N; i++) {
        x0 = x[i - T + 2];
        y[i] = x[i] + MULT16_32_Q15(g10, x2) + MULT16_32_Q15(g11, ADD32(x1, x3)) + MULT16_32_Q15(g12, ADD32(x0, x4));
        y[i] = SATURATE(y[i], SIG_SAT);
        x4 = x3;
        x3 = x2;
        x2 = x1;
        x1 = x0;
    }
}
//----------------------------------------------------------------------------------------------------------------------

void comb_filter(int32_t *y, int32_t *x, int T0, int T1, int N, int16_t g0, int16_t g1, int tapset0, int tapset1,
                 const int16_t *window, int overlap, int arch) {
    int i;
    /* printf ("%d %d %f %f\n", T0, T1, g0, g1); */
    int16_t g00, g01, g02, g10, g11, g12;
    int32_t x0, x1, x2, x3, x4;
    static const int16_t gains[3][3] = {
        {QCONST16(0.3066406250f, 15), QCONST16(0.2170410156f, 15), QCONST16(0.1296386719f, 15)},
        {QCONST16(0.4638671875f, 15), QCONST16(0.2680664062f, 15), QCONST16(0.f, 15)},
        {QCONST16(0.7998046875f, 15), QCONST16(0.1000976562f, 15), QCONST16(0.f, 15)}};

    if (g0 == 0 && g1 == 0) {
        /* OPT: Happens to work without the OPUS_MOVE(), but only because the current encoder already copies x to y */
        if (x != y)
            OPUS_MOVE(y, x, N);
        return;
    }
    /* When the gain is zero, T0 and/or T1 is set to zero. We need
       to have then be at least 2 to avoid processing garbage data. */
    T0 = IMAX(T0, COMBFILTER_MINPERIOD);
    T1 = IMAX(T1, COMBFILTER_MINPERIOD);
    g00 = MULT16_16_P15(g0, gains[tapset0][0]);
    g01 = MULT16_16_P15(g0, gains[tapset0][1]);
    g02 = MULT16_16_P15(g0, gains[tapset0][2]);
    g10 = MULT16_16_P15(g1, gains[tapset1][0]);
    g11 = MULT16_16_P15(g1, gains[tapset1][1]);
    g12 = MULT16_16_P15(g1, gains[tapset1][2]);
    x1 = x[-T1 + 1];
    x2 = x[-T1];
    x3 = x[-T1 - 1];
    x4 = x[-T1 - 2];
    /* If the filter didn't change, we don't need the overlap */
    if (g0 == g1 && T0 == T1 && tapset0 == tapset1)
        overlap = 0;
    for (i = 0; i < overlap; i++) {
        int16_t f;
        x0 = x[i - T1 + 2];
        f = MULT16_16_Q15(window[i], window[i]);
        y[i] = x[i] + MULT16_32_Q15(MULT16_16_Q15((32767 - f), g00), x[i - T0]) + MULT16_32_Q15(MULT16_16_Q15((32767 - f), g01), ADD32(x[i - T0 + 1], x[i - T0 - 1])) + MULT16_32_Q15(MULT16_16_Q15((32767 - f), g02), ADD32(x[i - T0 + 2], x[i - T0 - 2])) + MULT16_32_Q15(MULT16_16_Q15(f, g10), x2) + MULT16_32_Q15(MULT16_16_Q15(f, g11), ADD32(x1, x3)) + MULT16_32_Q15(MULT16_16_Q15(f, g12), ADD32(x0, x4));
        y[i] = SATURATE(y[i], SIG_SAT);
        x4 = x3;
        x3 = x2;
        x2 = x1;
        x1 = x0;
    }
    if (g1 == 0) {
        /* OPT: Happens to work without the OPUS_MOVE(), but only because the current encoder already copies x to y */
        if (x != y)
            OPUS_MOVE(y + overlap, x + overlap, N - overlap);
        return;
    }

    /* Compute the part with the constant filter. */
    comb_filter_const(y + i, x + i, T1, N - i, g10, g11, g12, arch);
}
//----------------------------------------------------------------------------------------------------------------------

/* TF change table. Positive values mean better frequency resolution (longer effective window), whereas negative values
   mean better time resolution (shorter effective window). The second index is computed as:
   4*isTransient + 2*tf_select + per_band_flag */
const signed char tf_select_table[4][8] = {
    /*isTransient=0     isTransient=1 */
    {0, -1, 0, -1, 0, -1, 0, -1}, /* 2.5 ms */
    {0, -1, 0, -2, 1, 0, 1, -1},  /* 5 ms */
    {0, -2, 0, -3, 2, 0, 1, -1},  /* 10 ms */
    {0, -2, 0, -3, 3, 0, 1, -1},  /* 20 ms */
};
//----------------------------------------------------------------------------------------------------------------------

void init_caps(const CELTMode *m, int *cap, int LM, int C) {
    int i;
    for (i = 0; i < m->nbEBands; i++)
    {
        int N;
        N = (m->eBands[i + 1] - m->eBands[i]) << LM;
        cap[i] = (m->cache.caps[m->nbEBands * (2 * LM + C - 1) + i] + 64) * C * N >> 2;
    }
}
//----------------------------------------------------------------------------------------------------------------------

const char *opus_strerror(int error) {
    static const char *const error_strings[8] = {
        "success",
        "invalid argument",
        "buffer too small",
        "internal error",
        "corrupted stream",
        "request not implemented",
        "invalid state",
        "memory allocation failed"};
    if (error > 0 || error < -7)
        return "unknown error";
    else
        return error_strings[-error];
}
//----------------------------------------------------------------------------------------------------------------------

int hysteresis_decision(int16_t val, const int16_t *thresholds, const int16_t *hysteresis, int N, int prev) {
    int i;
    for (i = 0; i < N; i++) {
        if (val < thresholds[i])  break;
    }
    if (i > prev && val < thresholds[prev] + hysteresis[prev])  i = prev;
    if (i < prev && val > thresholds[prev - 1] - hysteresis[prev - 1]) i = prev;
    return i;
}
//----------------------------------------------------------------------------------------------------------------------

uint32_t celt_lcg_rand(uint32_t seed) {
    return 1664525 * seed + 1013904223;
}
//----------------------------------------------------------------------------------------------------------------------

/* This is a cos() approximation designed to be bit-exact on any platform. Bit exactness
   with this approximation is important because it has an impact on the bit allocation */
int16_t bitexact_cos(int16_t x) {
    int32_t tmp;
    int16_t x2;
    tmp = (4096 + ((int32_t)(x) * (x))) >> 13;
    assert(tmp <= 32767);
    x2 = tmp;
    x2 = (32767 - x2) + FRAC_MUL16(x2, (-7651 + FRAC_MUL16(x2, (8277 + FRAC_MUL16(-626, x2)))));
    assert(x2 <= 32766);
    return 1 + x2;
}
//----------------------------------------------------------------------------------------------------------------------

int bitexact_log2tan(int isin, int icos) {
    int lc;
    int ls;
    lc = EC_ILOG(icos);
    ls = EC_ILOG(isin);
    icos <<= 15 - lc;
    isin <<= 15 - ls;
    return (ls - lc) * (1 << 11) + FRAC_MUL16(isin, FRAC_MUL16(isin, -2597) + 7932) - FRAC_MUL16(icos, FRAC_MUL16(icos, -2597) + 7932);
}
//----------------------------------------------------------------------------------------------------------------------

/* Compute the amplitude (sqrt energy) in each of the bands */
void compute_band_energies(const CELTMode *m, const int32_t *X, int32_t *bandE, int end, int C, int LM, int arch) {
    int i, c, N;
    const int16_t *eBands = m->eBands;
    (void)arch;
    N = m->shortMdctSize << LM;
    c = 0;
    do {
        for (i = 0; i < end; i++) {
            int j;
            int32_t maxval = 0;
            int32_t sum = 0;

            maxval = celt_maxabs32(&X[c * N + (eBands[i] << LM)], (eBands[i + 1] - eBands[i]) << LM);
            if (maxval > 0) {
                int shift = celt_ilog2(maxval) - 14 + (((m->logN[i] >> BITRES) + LM + 1) >> 1);
                j = eBands[i] << LM;
                if (shift > 0){
                    do {
                        sum = MAC16_16(sum, EXTRACT16(SHR32(X[j + c * N], shift)),
                                       EXTRACT16(SHR32(X[j + c * N], shift)));
                    } while (++j < eBands[i + 1] << LM);
                }
                else {
                    do {
                        sum = MAC16_16(sum, EXTRACT16(SHL32(X[j + c * N], -shift)),
                                       EXTRACT16(SHL32(X[j + c * N], -shift)));
                    } while (++j < eBands[i + 1] << LM);
                }
                /* We're adding one here to ensure the normalized band isn't larger than unity norm */
                bandE[i + c * m->nbEBands] = EPSILON + VSHR32(EXTEND32(celt_sqrt(sum)), -shift);
            }
            else {
                bandE[i + c * m->nbEBands] = EPSILON;
            }
            /*printf ("%f ", bandE[i+c*m->nbEBands]);*/
        }
    } while (++c < C);
    /*printf ("\n");*/
}
//----------------------------------------------------------------------------------------------------------------------

/* Normalise each band such that the energy is one. */
void normalise_bands(const CELTMode *m, const int32_t *__restrict__ freq, int16_t *__restrict__ X, const int32_t *bandE, int end, int C, int M)
{
    int i, c, N;
    const int16_t *eBands = m->eBands;
    N = M * m->shortMdctSize;
    c = 0;
    do
    {
        i = 0;
        do
        {
            int16_t g;
            int j, shift;
            int16_t E;
            shift = celt_zlog2(bandE[i + c * m->nbEBands]) - 13;
            E = VSHR32(bandE[i + c * m->nbEBands], shift);
            g = EXTRACT16(celt_rcp(SHL32(E, 3)));
            j = M * eBands[i];
            do
            {
                X[j + c * N] = MULT16_16_Q15(VSHR32(freq[j + c * N], shift - 1), g);
            } while (++j < M * eBands[i + 1]);
        } while (++i < end);
    } while (++c < C);
}
//----------------------------------------------------------------------------------------------------------------------

/* De-normalise the energy to produce the synthesis from the unit-energy bands */
void denormalise_bands(const CELTMode *m, const int16_t *__restrict__ X, int32_t *__restrict__ freq,
                       const int16_t *bandLogE, int start, int end, int M, int downsample, int silence) {
    int i, N;
    int bound;
    int32_t *__restrict__ f;
    const int16_t *__restrict__ x;
    const int16_t *eBands = m->eBands;
    N = M * m->shortMdctSize;
    bound = M * eBands[end];
    if (downsample != 1) bound = IMIN(bound, N / downsample);
    if (silence) {
        bound = 0;
        start = end = 0;
    }
    f = freq;
    x = X + M * eBands[start];
    for (i = 0; i < M * eBands[start]; i++)
        *f++ = 0;
    for (i = start; i < end; i++) {
        int j, band_end;
        int16_t g;
        int16_t lg;

        int shift;

        j = M * eBands[i];
        band_end = M * eBands[i + 1];
        lg = SATURATE16(ADD32(bandLogE[i], SHL32((int32_t)eMeans[i], 6)));

        /* Handle the integer part of the log energy */
        shift = 16 - (lg >> DB_SHIFT);
        if (shift > 31) {
            shift = 0;
            g = 0;
        }
        else {
            /* Handle the fractional part. */
            g = celt_exp2_frac(lg & ((1 << DB_SHIFT) - 1));
        }
        /* Handle extreme gains with negative shift. */
        if (shift < 0) {
            /* For shift <= -2 and g > 16384 we'd be likely to overflow, so we're
               capping the gain here, which is equivalent to a cap of 18 on lg.
               This shouldn't trigger unless the bitstream is already corrupted. */
            if (shift <= -2) {
                g = 16384;
                shift = -2;
            }
            do {
                *f++ = SHL32(MULT16_16(*x++, g), -shift);
            } while (++j < band_end);
        }
        else

            /* Be careful of the fixed-point "else" just above when changing this code */
            do {
                *f++ = SHR32(MULT16_16(*x++, g), shift);
            } while (++j < band_end);
    }
    celt_assert(start <= end);
    OPUS_CLEAR(&freq[bound], N - bound);
}
//----------------------------------------------------------------------------------------------------------------------

/* This prevents energy collapse for transients with multiple short MDCTs */
void anti_collapse(const CELTMode *m, int16_t *X_, unsigned char *collapse_masks, int LM, int C, int size, int start,
                   int end, const int16_t *logE, const int16_t *prev1logE, const int16_t *prev2logE, const int *pulses,
                   uint32_t seed, int arch){
    int c, i, j, k;
    for (i = start; i < end; i++) {
        int N0;
        int16_t thresh, sqrt_1;
        int depth;

        int shift;
        int32_t thresh32;

        N0 = m->eBands[i + 1] - m->eBands[i];
        /* depth in 1/8 bits */
        assert(pulses[i] >= 0);
        depth = celt_udiv(1 + pulses[i], (m->eBands[i + 1] - m->eBands[i])) >> LM;

        thresh32 = SHR32(celt_exp2(-SHL16(depth, 10 - BITRES)), 1);
        thresh = MULT16_32_Q15(QCONST16(0.5f, 15), MIN32(32767, thresh32)); {
            int32_t t;
            t = N0 << LM;
            shift = celt_ilog2(t) >> 1;
            t = SHL32(t, (7 - shift) << 1);
            sqrt_1 = celt_rsqrt_norm(t);
        }

        c = 0;
        do {
            int16_t *X;
            int16_t prev1;
            int16_t prev2;
            int32_t Ediff;
            int16_t r;
            int renormalize = 0;
            prev1 = prev1logE[c * m->nbEBands + i];
            prev2 = prev2logE[c * m->nbEBands + i];
            if (C == 1) {
                prev1 = MAX16(prev1, prev1logE[m->nbEBands + i]);
                prev2 = MAX16(prev2, prev2logE[m->nbEBands + i]);
            }
            Ediff = EXTEND32(logE[c * m->nbEBands + i]) - EXTEND32(MIN16(prev1, prev2));
            Ediff = MAX32(0, Ediff);

            if (Ediff < 16384) {
                int32_t r32 = SHR32(celt_exp2(-EXTRACT16(Ediff)), 1);
                r = 2 * MIN16(16383, r32);
            }
            else {
                r = 0;
            }
            if (LM == 3)
                r = MULT16_16_Q14(23170, MIN32(23169, r));
            r = SHR16(MIN16(thresh, r), 1);
            r = SHR32(MULT16_16_Q15(sqrt_1, r), shift);

            X = X_ + c * size + (m->eBands[i] << LM);
            for (k = 0; k < 1 << LM; k++) {
                /* Detect collapse */
                if (!(collapse_masks[i * C + c] & 1 << k)) {
                    /* Fill with noise */
                    for (j = 0; j < N0; j++) {
                        seed = celt_lcg_rand(seed);
                        X[(j << LM) + k] = (seed & 0x8000 ? r : -r);
                    }
                    renormalize = 1;
                }
            }
            /* We just added some energy, so we need to renormalise */
            if (renormalize)
                renormalise_vector(X, N0 << LM, 32767, arch);
        } while (++c < C);
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Compute the weights to use for optimizing normalized distortion across channels. We use the amplitude to weight
   square distortion, which means that we use the square root of the value we would have been using if we wanted to
   minimize the MSE in the non-normalized domain. This roughly corresponds to some quick-and-dirty perceptual
   experiments I ran to measure inter-aural masking (there doesn't seem to be any published data on the topic). */
static void compute_channel_weights(int32_t Ex, int32_t Ey, int16_t w[2]) {
    int32_t minE;

    int shift;

    minE = MIN32(Ex, Ey);
    /* Adjustment to make the weights a bit more conservative. */
    Ex = ADD32(Ex, minE / 3);
    Ey = ADD32(Ey, minE / 3);

    shift = celt_ilog2(EPSILON + MAX32(Ex, Ey)) - 14;

    w[0] = VSHR32(Ex, shift);
    w[1] = VSHR32(Ey, shift);
}
//----------------------------------------------------------------------------------------------------------------------

static void intensity_stereo(const CELTMode *m, int16_t *__restrict__ X, const int16_t *__restrict__ Y,
                             const int32_t *bandE, int bandID, int N){
    int i = bandID;
    int j;
    int16_t a1, a2;
    int16_t left, right;
    int16_t norm;

    int shift = celt_zlog2(MAX32(bandE[i], bandE[i + m->nbEBands])) - 13;

    left = VSHR32(bandE[i], shift);
    right = VSHR32(bandE[i + m->nbEBands], shift);
    norm = EPSILON + celt_sqrt(EPSILON + MULT16_16(left, left) + MULT16_16(right, right));
    a1 = DIV32_16(SHL32(EXTEND32(left), 14), norm);
    a2 = DIV32_16(SHL32(EXTEND32(right), 14), norm);
    for (j = 0; j < N; j++) {
        int16_t r, l;
        l = X[j];
        r = Y[j];
        X[j] = EXTRACT16(SHR32(MAC16_16(MULT16_16(a1, l), a2, r), 14));
        /* Side is not encoded, no need to calculate */
    }
}
//----------------------------------------------------------------------------------------------------------------------

static void stereo_split(int16_t *__restrict__ X, int16_t *__restrict__ Y, int N) {
    int j;
    for (j = 0; j < N; j++) {
        int32_t r, l;
        l = MULT16_16(QCONST16(.70710678f, 15), X[j]);
        r = MULT16_16(QCONST16(.70710678f, 15), Y[j]);
        X[j] = EXTRACT16(SHR32(ADD32(l, r), 15));
        Y[j] = EXTRACT16(SHR32(SUB32(r, l), 15));
    }
}
//----------------------------------------------------------------------------------------------------------------------

static void stereo_merge(int16_t *__restrict__ X, int16_t *__restrict__ Y, int16_t mid, int N, int arch){
    int j;
    int32_t xp = 0, side = 0;
    int32_t El, Er;
    int16_t mid2;

    int kl, kr;

    int32_t t, lgain, rgain;

    /* Compute the norm of X+Y and X-Y as |X|^2 + |Y|^2 +/- sum(xy) */
    dual_inner_prod(Y, X, Y, N, &xp, &side, arch);
    /* Compensating for the mid normalization */
    xp = MULT16_32_Q15(mid, xp);
    /* mid and side are in Q15, not Q14 like X and Y */
    mid2 = SHR16(mid, 1);
    El = MULT16_16(mid2, mid2) + side - 2 * xp;
    Er = MULT16_16(mid2, mid2) + side + 2 * xp;
    if (Er < QCONST32(6e-4f, 28) || El < QCONST32(6e-4f, 28)) {
        OPUS_COPY(Y, X, N);
        return;
    }

    kl = celt_ilog2(El) >> 1;
    kr = celt_ilog2(Er) >> 1;

    t = VSHR32(El, (kl - 7) << 1);
    lgain = celt_rsqrt_norm(t);
    t = VSHR32(Er, (kr - 7) << 1);
    rgain = celt_rsqrt_norm(t);

    if (kl < 7)  kl = 7;
    if (kr < 7)  kr = 7;

    for (j = 0; j < N; j++) {
        int16_t r, l;
        /* Apply mid scaling (side is already scaled) */
        l = MULT16_16_P15(mid, X[j]);
        r = Y[j];
        X[j] = EXTRACT16(PSHR32(MULT16_16(lgain, SUB16(l, r)), kl + 1));
        Y[j] = EXTRACT16(PSHR32(MULT16_16(rgain, ADD16(l, r)), kr + 1));
    }
}
//----------------------------------------------------------------------------------------------------------------------

/* Decide whether we should spread the pulses in the current frame */
int spreading_decision(const CELTMode *m, const int16_t *X, int *average, int last_decision, int *hf_average,
                       int *tapset_decision, int update_hf, int end, int C, int M, const int *spread_weight) {
    int i, c, N0;
    int sum = 0, nbBands = 0;
    const int16_t *__restrict__ eBands = m->eBands;
    int decision;
    int hf_sum = 0;

    celt_assert(end > 0);

    N0 = M * m->shortMdctSize;

    if (M * (eBands[end] - eBands[end - 1]) <= 8)
        return SPREAD_NONE;
    c = 0;
    do {
        for (i = 0; i < end; i++) {
            int j, N, tmp = 0;
            int tcount[3] = {0, 0, 0};
            const int16_t *__restrict__ x = X + M * eBands[i] + c * N0;
            N = M * (eBands[i + 1] - eBands[i]);
            if (N <= 8)
                continue;
            /* Compute rough CDF of |x[j]| */
            for (j = 0; j < N; j++) {
                int32_t x2N; /* Q13 */

                x2N = MULT16_16(MULT16_16_Q15(x[j], x[j]), N);
                if (x2N < QCONST16(0.25f, 13))
                    tcount[0]++;
                if (x2N < QCONST16(0.0625f, 13))
                    tcount[1]++;
                if (x2N < QCONST16(0.015625f, 13))
                    tcount[2]++;
            }

            /* Only include four last bands (8 kHz and up) */
            if (i > m->nbEBands - 4)
                hf_sum += celt_udiv(32 * (tcount[1] + tcount[0]), N);
            tmp = (2 * tcount[2] >= N) + (2 * tcount[1] >= N) + (2 * tcount[0] >= N);
            sum += tmp * spread_weight[i];
            nbBands += spread_weight[i];
        }
    } while (++c < C);

    if (update_hf) {
        if (hf_sum)
            hf_sum = celt_udiv(hf_sum, C * (4 - m->nbEBands + end));
        *hf_average = (*hf_average + hf_sum) >> 1;
        hf_sum = *hf_average;
        if (*tapset_decision == 2)
            hf_sum += 4;
        else if (*tapset_decision == 0)
            hf_sum -= 4;
        if (hf_sum > 22)
            *tapset_decision = 2;
        else if (hf_sum > 18)
            *tapset_decision = 1;
        else
            *tapset_decision = 0;
    }
    /*printf("%d %d %d\n", hf_sum, *hf_average, *tapset_decision);*/
    celt_assert(nbBands > 0); /* end has to be non-zero */
    celt_assert(sum >= 0);
    sum = celt_udiv((int32_t)sum << 8, nbBands);
    /* Recursive averaging */
    sum = (sum + *average) >> 1;
    *average = sum;
    /* Hysteresis */
    sum = (3 * sum + (((3 - last_decision) << 7) + 64) + 2) >> 2;
    if (sum < 80) {
        decision = SPREAD_AGGRESSIVE;
    }
    else if (sum < 256) {
        decision = SPREAD_NORMAL;
    }
    else if (sum < 384) {
        decision = SPREAD_LIGHT;
    }
    else {
        decision = SPREAD_NONE;
    }

    return decision;
}
//----------------------------------------------------------------------------------------------------------------------

/* Indexing table for converting from natural Hadamard to ordery Hadamard. This is essentially a bit-reversed Gray,
   on top of which we've added an inversion of the order because we want the DC at the end rather than the beginning.
   The lines are for N=2, 4, 8, 16 */
static const int ordery_table[] = {
    1,
    0,
    3,
    0,
    2,
    1,
    7,
    0,
    4,
    3,
    6,
    1,
    5,
    2,
    15,
    0,
    8,
    7,
    12,
    3,
    11,
    4,
    14,
    1,
    9,
    6,
    13,
    2,
    10,
    5,
};
//----------------------------------------------------------------------------------------------------------------------

static void deinterleave_hadamard(int16_t *X, int N0, int stride, int hadamard){
    int i, j;
    VARDECL(int16_t, tmp);
    int N;
    SAVE_STACK;
    N = N0 * stride;
    ALLOC(tmp, N, int16_t);
    celt_assert(stride > 0);
    if (hadamard) {
        const int *ordery = ordery_table + stride - 2;
        for (i = 0; i < stride; i++) {
            for (j = 0; j < N0; j++)
                tmp[ordery[i] * N0 + j] = X[j * stride + i];
        }
    }
    else {
        for (i = 0; i < stride; i++)
            for (j = 0; j < N0; j++)
                tmp[i * N0 + j] = X[j * stride + i];
    }
    OPUS_COPY(X, tmp, N);
}
//----------------------------------------------------------------------------------------------------------------------

static void interleave_hadamard(int16_t *X, int N0, int stride, int hadamard){
    int i, j;
    VARDECL(int16_t, tmp);
    int N;
    SAVE_STACK;
    N = N0 * stride;
    ALLOC(tmp, N, int16_t);
    if (hadamard) {
        const int *ordery = ordery_table + stride - 2;
        for (i = 0; i < stride; i++)
            for (j = 0; j < N0; j++)
                tmp[j * stride + i] = X[ordery[i] * N0 + j];
    }
    else {
        for (i = 0; i < stride; i++)
            for (j = 0; j < N0; j++)
                tmp[j * stride + i] = X[i * N0 + j];
    }
    OPUS_COPY(X, tmp, N);
}
//----------------------------------------------------------------------------------------------------------------------

void haar1(int16_t *X, int N0, int stride) {
    int i, j;
    N0 >>= 1;
    for (i = 0; i < stride; i++)
        for (j = 0; j < N0; j++) {
            int32_t tmp1, tmp2;
            tmp1 = MULT16_16(QCONST16(.70710678f, 15), X[stride * 2 * j + i]);
            tmp2 = MULT16_16(QCONST16(.70710678f, 15), X[stride * (2 * j + 1) + i]);
            X[stride * 2 * j + i] = EXTRACT16(PSHR32(ADD32(tmp1, tmp2), 15));
            X[stride * (2 * j + 1) + i] = EXTRACT16(PSHR32(SUB32(tmp1, tmp2), 15));
        }
}
//----------------------------------------------------------------------------------------------------------------------

static int compute_qn(int N, int b, int offset, int pulse_cap, int stereo) {
    static const int16_t exp2_table8[8] =
        {16384, 17866, 19483, 21247, 23170, 25267, 27554, 30048};
    int qn, qb;
    int N2 = 2 * N - 1;
    if (stereo && N == 2)
        N2--;
    /* The upper limit ensures that in a stereo split with itheta==16384, we'll
        always have enough bits left over to code at least one pulse in the
        side; otherwise it would collapse, since it doesn't get folded. */
    qb = celt_sudiv(b + N2 * offset, N2);
    qb = IMIN(b - pulse_cap - (4 << BITRES), qb);

    qb = IMIN(8 << BITRES, qb);

    if (qb < (1 << BITRES >> 1)) {
        qn = 1;
    }
    else {
        qn = exp2_table8[qb & 0x7] >> (14 - (qb >> BITRES));
        qn = (qn + 1) >> 1 << 1;
    }
    celt_assert(qn <= 256);
    return qn;
}
//----------------------------------------------------------------------------------------------------------------------

static void compute_theta(struct band_ctx *ctx, struct split_ctx *sctx, int16_t *X, int16_t *Y, int N, int *b, int B,
                          int __B0, int LM, int stereo, int *fill) {
    int qn;
    int itheta = 0;
    int delta;
    int imid, iside;
    int qalloc;
    int pulse_cap;
    int offset;
    int32_t tell;
    int inv = 0;
    int encode;
    const CELTMode *m;
    int i;
    int intensity;
    ec_ctx *ec;
    const int32_t *bandE;

    encode = ctx->encode;
    m = ctx->m;
    i = ctx->i;
    intensity = ctx->intensity;
    ec = ctx->ec;
    bandE = ctx->bandE;

    /* Decide on the resolution to give to the split parameter theta */
    pulse_cap = m->logN[i] + LM * (1 << BITRES);
    offset = (pulse_cap >> 1) - (stereo && N == 2 ? QTHETA_OFFSET_TWOPHASE : QTHETA_OFFSET);
    qn = compute_qn(N, *b, offset, pulse_cap, stereo);
    if (stereo && i >= intensity)
        qn = 1;
    if (encode) {
        /* theta is the atan() of the ratio between the (normalized) side and mid. With just that parameter,
           we can re-scale both mid and side because we know that 1) they have unit norm and 2) they are orthogonal. */
        itheta = stereo_itheta(X, Y, stereo, N, ctx->arch);
    }
    tell = ec_tell_frac(ec);
    if (qn != 1) {
        if (encode) {
            if (!stereo || ctx->theta_round == 0) {
                itheta = (itheta * (int32_t)qn + 8192) >> 14;
                if (!stereo && ctx->avoid_split_noise && itheta > 0 && itheta < qn) {
                    /* Check if the selected value of theta will cause the bit allocation to inject noise on one side.
                       If so, make sure the energy of that side is zero. */
                    int unquantized = celt_udiv((int32_t)itheta * 16384, qn);
                    imid = bitexact_cos((int16_t)unquantized);
                    iside = bitexact_cos((int16_t)(16384 - unquantized));
                    delta = FRAC_MUL16((N - 1) << 7, bitexact_log2tan(iside, imid));
                    if (delta > *b)
                        itheta = qn;
                    else if (delta < -*b)
                        itheta = 0;
                }
            }
            else {
                int down;
                /* Bias quantization towards itheta=0 and itheta=16384. */
                int bias = itheta > 8192 ? 32767 / qn : -32767 / qn;
                down = IMIN(qn - 1, IMAX(0, (itheta * (int32_t)qn + bias) >> 14));
                if (ctx->theta_round < 0)
                    itheta = down;
                else
                    itheta = down + 1;
            }
        }
        /* Entropy coding of the angle. We use a uniform pdf for the time split, a step for stereo,
           and a triangular one for the rest. */
        if (stereo && N > 2) {
            int p0 = 3;
            int x = itheta;
            int x0 = qn / 2;
            int ft = p0 * (x0 + 1) + x0;
            /* Use a probability of p0 up to itheta=8192 and then use 1 after */
            if (encode) {
                ec_encode(ec, x <= x0 ?
                              p0 * x : (x - 1 - x0) + (x0 + 1) * p0, x <= x0 ?
                                  p0 * (x + 1) : (x - x0) + (x0 + 1) * p0, ft);
            }
            else {
                int fs;
                fs = ec_decode(ec, ft);
                if (fs < (x0 + 1) * p0)
                    x = fs / p0;
                else
                    x = x0 + 1 + (fs - (x0 + 1) * p0);
                ec_dec_update(ec, x <= x0 ? p0 * x : (x - 1 - x0) + (x0 + 1) * p0, x <= x0 ? p0 * (x + 1) : (x - x0) + (x0 + 1) * p0, ft);
                itheta = x;
            }
        }
        else if (__B0 > 1 || stereo) {
            /* Uniform pdf */
            if (encode)
                ec_enc_uint(ec, itheta, qn + 1);
            else
                itheta = ec_dec_uint(ec, qn + 1);
        }
        else {
            int fs = 1, ft;
            ft = ((qn >> 1) + 1) * ((qn >> 1) + 1);
            if (encode) {
                int fl;

                fs = itheta <= (qn >> 1) ? itheta + 1 : qn + 1 - itheta;
                fl = itheta <= (qn >> 1) ? itheta * (itheta + 1) >> 1 : ft - ((qn + 1 - itheta) * (qn + 2 - itheta) >> 1);

                ec_encode(ec, fl, fl + fs, ft);
            }
            else {
                /* Triangular pdf */
                int fl = 0;
                int fm;
                fm = ec_decode(ec, ft);

                if (fm < ((qn >> 1) * ((qn >> 1) + 1) >> 1))
                {
                    itheta = (isqrt32(8 * (uint32_t)fm + 1) - 1) >> 1;
                    fs = itheta + 1;
                    fl = itheta * (itheta + 1) >> 1;
                }
                else
                {
                    itheta = (2 * (qn + 1) - isqrt32(8 * (uint32_t)(ft - fm - 1) + 1)) >> 1;
                    fs = qn + 1 - itheta;
                    fl = ft - ((qn + 1 - itheta) * (qn + 2 - itheta) >> 1);
                }

                ec_dec_update(ec, fl, fl + fs, ft);
            }
        }
        celt_assert(itheta >= 0);
        itheta = celt_udiv((int32_t)itheta * 16384, qn);
        if (encode && stereo) {
            if (itheta == 0)
                intensity_stereo(m, X, Y, bandE, i, N);
            else
                stereo_split(X, Y, N);
        }
        /* NOTE: Renormalising X and Y *may* help fixed-point a bit at very high rate.
                 Let's do that at higher complexity */
    }
    else if (stereo) {
        if (encode) {
            inv = itheta > 8192 && !ctx->disable_inv;
            if (inv)  {
                int j;
                for (j = 0; j < N; j++)
                    Y[j] = -Y[j];
            }
            intensity_stereo(m, X, Y, bandE, i, N);
        }
        if (*b > 2 << BITRES && ctx->remaining_bits > 2 << BITRES) {
            if (encode)
                ec_enc_bit_logp(ec, inv, 2);
            else
                inv = ec_dec_bit_logp(ec, 2);
        }
        else
            inv = 0;
        /* inv flag override to avoid problems with downmixing. */
        if (ctx->disable_inv)
            inv = 0;
        itheta = 0;
    }
    qalloc = ec_tell_frac(ec) - tell;
    *b -= qalloc;

    if (itheta == 0) {
        imid = 32767;
        iside = 0;
        *fill &= (1 << B) - 1;
        delta = -16384;
    }
    else if (itheta == 16384){
        imid = 0;
        iside = 32767;
        *fill &= ((1 << B) - 1) << B;
        delta = 16384;
    }
    else {
        imid = bitexact_cos((int16_t)itheta);
        iside = bitexact_cos((int16_t)(16384 - itheta));
        /* This is the mid vs side allocation that minimizes squared error
           in that band. */
        delta = FRAC_MUL16((N - 1) << 7, bitexact_log2tan(iside, imid));
    }

    sctx->inv = inv;
    sctx->imid = imid;
    sctx->iside = iside;
    sctx->delta = delta;
    sctx->itheta = itheta;
    sctx->qalloc = qalloc;
}
//----------------------------------------------------------------------------------------------------------------------

static unsigned quant_band_n1(struct band_ctx *ctx, int16_t *X, int16_t *Y, int b,  int16_t *lowband_out) {
    int c;
    int stereo;
    int16_t *x = X;
    int encode;
    ec_ctx *ec;

    encode = ctx->encode;
    ec = ctx->ec;

    stereo = Y != NULL;
    c = 0;
    do {
        int sign = 0;
        if (ctx->remaining_bits >= 1 << BITRES) {
            if (encode) {
                sign = x[0] < 0;
                ec_enc_bits(ec, sign, 1);
            }
            else {
                sign = ec_dec_bits(ec, 1);
            }
            ctx->remaining_bits -= 1 << BITRES;
            b -= 1 << BITRES;
        }
        if (ctx->resynth)
            x[0] = sign ? -NORM_SCALING : NORM_SCALING;
        x = Y;
    } while (++c < 1 + stereo);
    if (lowband_out)
        lowband_out[0] = SHR16(X[0], 4);
    return 1;
}
//----------------------------------------------------------------------------------------------------------------------

/* This function is responsible for encoding and decoding a mono partition. It can split the band in two and transmit
   the energy difference with the two half-bands. It can be called recursively so bands can end up being
   split in 8 parts. */
static unsigned quant_partition(struct band_ctx *ctx, int16_t *X, int N, int b, int B, int16_t *lowband, int LM,
                                int16_t gain, int fill){
    const unsigned char *cache;
    int q;
    int curr_bits;
    int imid = 0, iside = 0;
    int _B0 = B;
    int16_t mid = 0, side = 0;
    unsigned cm = 0;
    int16_t *Y = NULL;
    int encode;
    const CELTMode *m;
    int i;
    int spread;
    ec_ctx *ec;

    encode = ctx->encode;
    m = ctx->m;
    i = ctx->i;
    spread = ctx->spread;
    ec = ctx->ec;

    /* If we need 1.5 more bit than we can produce, split the band in two. */
    cache = m->cache.bits + m->cache.index[(LM + 1) * m->nbEBands + i];
    if (LM != -1 && b > cache[cache[0]] + 12 && N > 2) {
        int mbits, sbits, delta;
        int itheta;
        int qalloc;
        struct split_ctx sctx;
        int16_t *next_lowband2 = NULL;
        int32_t rebalance;

        N >>= 1;
        Y = X + N;
        LM -= 1;
        if (B == 1)
            fill = (fill & 1) | (fill << 1);
        B = (B + 1) >> 1;

        compute_theta(ctx, &sctx, X, Y, N, &b, B, _B0, LM, 0, &fill);
        imid = sctx.imid;
        iside = sctx.iside;
        delta = sctx.delta;
        itheta = sctx.itheta;
        qalloc = sctx.qalloc;

        mid = imid;
        side = iside;

        /* Give more bits to low-energy MDCTs than they would otherwise deserve */
        if (_B0 > 1 && (itheta & 0x3fff)) {
            if (itheta > 8192)
                /* Rough approximation for pre-echo masking */
                delta -= delta >> (4 - LM);
            else
                /* Corresponds to a forward-masking slope of 1.5 dB per 10 ms */
                delta = IMIN(0, delta + (N << BITRES >> (5 - LM)));
        }
        mbits = IMAX(0, IMIN(b, (b - delta) / 2));
        sbits = b - mbits;
        ctx->remaining_bits -= qalloc;

        if (lowband)
            next_lowband2 = lowband + N; /* >32-bit split case */

        rebalance = ctx->remaining_bits;
        if (mbits >= sbits)  {
            cm = quant_partition(ctx, X, N, mbits, B, lowband, LM,
                                 MULT16_16_P15(gain, mid), fill);
            rebalance = mbits - (rebalance - ctx->remaining_bits);
            if (rebalance > 3 << BITRES && itheta != 0)
                sbits += rebalance - (3 << BITRES);
            cm |= quant_partition(ctx, Y, N, sbits, B, next_lowband2, LM,
                                  MULT16_16_P15(gain, side), fill >> B)
                  << (_B0 >> 1);
        }
        else {
            cm = quant_partition(ctx, Y, N, sbits, B, next_lowband2, LM,
                                 MULT16_16_P15(gain, side), fill >> B)
                 << (_B0 >> 1);
            rebalance = sbits - (rebalance - ctx->remaining_bits);
            if (rebalance > 3 << BITRES && itheta != 16384)
                mbits += rebalance - (3 << BITRES);
            cm |= quant_partition(ctx, X, N, mbits, B, lowband, LM,
                                  MULT16_16_P15(gain, mid), fill);
        }
    }
    else {
        /* This is the basic no-split case */
        q = bits2pulses(m, i, LM, b);
        curr_bits = pulses2bits(m, i, LM, q);
        ctx->remaining_bits -= curr_bits;

        /* Ensures we can never bust the budget */
        while (ctx->remaining_bits < 0 && q > 0) {
            ctx->remaining_bits += curr_bits;
            q--;
            curr_bits = pulses2bits(m, i, LM, q);
            ctx->remaining_bits -= curr_bits;
        }

        if (q != 0) {
            int K = get_pulses(q);

            /* Finally do the actual quantization */
            if (encode)
            {
                cm = alg_quant(X, N, K, spread, B, ec, gain, ctx->resynth, ctx->arch);
            }
            else
            {
                cm = alg_unquant(X, N, K, spread, B, ec, gain);
            }
        }
        else {
            /* If there's no pulse, fill the band anyway */
            int j;
            if (ctx->resynth)
            {
                unsigned cm_mask;
                /* B can be as large as 16, so this shift might overflow an int on a
                   16-bit platform; use a long to get defined behavior.*/
                cm_mask = (unsigned)(1UL << B) - 1;
                fill &= cm_mask;
                if (!fill) {
                    OPUS_CLEAR(X, N);
                }
                else  {
                    if (lowband == NULL) {
                        /* Noise */
                        for (j = 0; j < N; j++) {
                            ctx->seed = celt_lcg_rand(ctx->seed);
                            X[j] = (int16_t)((int32_t)ctx->seed >> 20);
                        }
                        cm = cm_mask;
                    }
                    else {
                        /* Folded spectrum */
                        for (j = 0; j < N; j++) {
                            int16_t tmp;
                            ctx->seed = celt_lcg_rand(ctx->seed);
                            /* About 48 dB below the "normal" folding level */
                            tmp = QCONST16(1.0f / 256, 10);
                            tmp = (ctx->seed) & 0x8000 ? tmp : -tmp;
                            X[j] = lowband[j] + tmp;
                        }
                        cm = fill;
                    }
                    renormalise_vector(X, N, gain, ctx->arch);
                }
            }
        }
    }

    return cm;
}
//----------------------------------------------------------------------------------------------------------------------

/* This function is responsible for encoding and decoding a band for the mono case. */
static unsigned quant_band(struct band_ctx *ctx, int16_t *X, int N, int b, int B, int16_t *lowband, int LM,
                           int16_t *lowband_out, int16_t gain, int16_t *lowband_scratch, int fill) {
    int N0 = N;
    int N_B = N;
    int N__B0;
    int _B0 = B;
    int time_divide = 0;
    int recombine = 0;
    int longBlocks;
    unsigned cm = 0;
    int k;
    int encode;
    int tf_change;

    encode = ctx->encode;
    tf_change = ctx->tf_change;

    longBlocks = _B0 == 1;

    N_B = celt_udiv(N_B, B);

    /* Special case for one sample */
    if (N == 1) {
        return quant_band_n1(ctx, X, NULL, b, lowband_out);
    }

    if (tf_change > 0)
        recombine = tf_change;
    /* Band recombining to increase frequency resolution */

    if (lowband_scratch && lowband && (recombine || ((N_B & 1) == 0 && tf_change < 0) || _B0 > 1)) {
        OPUS_COPY(lowband_scratch, lowband, N);
        lowband = lowband_scratch;
    }

    for (k = 0; k < recombine; k++) {
        static const unsigned char bit_interleave_table[16] = {
            0, 1, 1, 1, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3};
        if (encode)
            haar1(X, N >> k, 1 << k);
        if (lowband)
            haar1(lowband, N >> k, 1 << k);
        fill = bit_interleave_table[fill & 0xF] | bit_interleave_table[fill >> 4] << 2;
    }
    B >>= recombine;
    N_B <<= recombine;

    /* Increasing the time resolution */
    while ((N_B & 1) == 0 && tf_change < 0) {
        if (encode)
            haar1(X, N_B, B);
        if (lowband)
            haar1(lowband, N_B, B);
        fill |= fill << B;
        B <<= 1;
        N_B >>= 1;
        time_divide++;
        tf_change++;
    }
    _B0 = B;
    N__B0 = N_B;

    /* Reorganize the samples in time order instead of frequency order */
    if (_B0 > 1) {
        if (encode)
            deinterleave_hadamard(X, N_B >> recombine, _B0 << recombine, longBlocks);
        if (lowband)
            deinterleave_hadamard(lowband, N_B >> recombine, _B0 << recombine, longBlocks);
    }

    cm = quant_partition(ctx, X, N, b, B, lowband, LM, gain, fill);

    /* This code is used by the decoder and by the resynthesis-enabled encoder */
    if (ctx->resynth) {
        /* Undo the sample reorganization going from time order to frequency order */
        if (_B0 > 1)
            interleave_hadamard(X, N_B >> recombine, _B0 << recombine, longBlocks);

        /* Undo time-freq changes that we did earlier */
        N_B = N__B0;
        B = _B0;
        for (k = 0; k < time_divide; k++) {
            B >>= 1;
            N_B <<= 1;
            cm |= cm >> B;
            haar1(X, N_B, B);
        }

        for (k = 0; k < recombine; k++) {
            static const unsigned char bit_deinterleave_table[16] = {
                0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
                0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF};
            cm = bit_deinterleave_table[cm];
            haar1(X, N0 >> k, 1 << k);
        }
        B <<= recombine;

        /* Scale output for later folding */
        if (lowband_out) {
            int j;
            int16_t n;
            n = celt_sqrt(SHL32(EXTEND32(N0), 22));
            for (j = 0; j < N0; j++)
                lowband_out[j] = MULT16_16_Q15(n, X[j]);
        }
        cm &= (1 << B) - 1;
    }
    return cm;
}
//----------------------------------------------------------------------------------------------------------------------

/* This function is responsible for encoding and decoding a band for the stereo case. */
static unsigned quant_band_stereo(struct band_ctx *ctx, int16_t *X, int16_t *Y, int N, int b, int B, int16_t *lowband,
                                  int LM, int16_t *lowband_out, int16_t *lowband_scratch, int fill) {
    int imid = 0, iside = 0;
    int inv = 0;
    int16_t mid = 0, side = 0;
    unsigned cm = 0;
    int mbits, sbits, delta;
    int itheta;
    int qalloc;
    struct split_ctx sctx;
    int orig_fill;
    int encode;
    ec_ctx *ec;

    encode = ctx->encode;
    ec = ctx->ec;

    /* Special case for one sample */
    if (N == 1){
        return quant_band_n1(ctx, X, Y, b, lowband_out);
    }

    orig_fill = fill;

    compute_theta(ctx, &sctx, X, Y, N, &b, B, B, LM, 1, &fill);
    inv = sctx.inv;
    imid = sctx.imid;
    iside = sctx.iside;
    delta = sctx.delta;
    itheta = sctx.itheta;
    qalloc = sctx.qalloc;

    mid = imid;
    side = iside;

    /* This is a special case for N=2 that only works for stereo and takes
       advantage of the fact that mid and side are orthogonal to encode
       the side with just one bit. */
    if (N == 2) {
        int c;
        int sign = 0;
        int16_t *x2, *y2;
        mbits = b;
        sbits = 0;
        /* Only need one bit for the side. */
        if (itheta != 0 && itheta != 16384)
            sbits = 1 << BITRES;
        mbits -= sbits;
        c = itheta > 8192;
        ctx->remaining_bits -= qalloc + sbits;

        x2 = c ? Y : X;
        y2 = c ? X : Y;
        if (sbits) {
            if (encode) {
                /* Here we only need to encode a sign for the side. */
                sign = x2[0] * y2[1] - x2[1] * y2[0] < 0;
                ec_enc_bits(ec, sign, 1);
            }
            else {
                sign = ec_dec_bits(ec, 1);
            }
        }
        sign = 1 - 2 * sign;
        /* We use orig_fill here because we want to fold the side, but if
           itheta==16384, we'll have cleared the low bits of fill. */
        cm = quant_band(ctx, x2, N, mbits, B, lowband, LM, lowband_out, 32767,
                        lowband_scratch, orig_fill);
        /* We don't split N=2 bands, so cm is either 1 or 0 (for a fold-collapse),
           and there's no need to worry about mixing with the other channel. */
        y2[0] = -sign * x2[1];
        y2[1] = sign * x2[0];
        if (ctx->resynth) {
            int16_t tmp;
            X[0] = MULT16_16_Q15(mid, X[0]);
            X[1] = MULT16_16_Q15(mid, X[1]);
            Y[0] = MULT16_16_Q15(side, Y[0]);
            Y[1] = MULT16_16_Q15(side, Y[1]);
            tmp = X[0];
            X[0] = SUB16(tmp, Y[0]);
            Y[0] = ADD16(tmp, Y[0]);
            tmp = X[1];
            X[1] = SUB16(tmp, Y[1]);
            Y[1] = ADD16(tmp, Y[1]);
        }
    }
    else {
        /* "Normal" split code */
        int32_t rebalance;

        mbits = IMAX(0, IMIN(b, (b - delta) / 2));
        sbits = b - mbits;
        ctx->remaining_bits -= qalloc;

        rebalance = ctx->remaining_bits;
        if (mbits >= sbits) {
            /* In stereo mode, we do not apply a scaling to the mid because we need the normalized
               mid for folding later. */
            cm = quant_band(ctx, X, N, mbits, B, lowband, LM, lowband_out, 32767,
                            lowband_scratch, fill);
            rebalance = mbits - (rebalance - ctx->remaining_bits);
            if (rebalance > 3 << BITRES && itheta != 0)
                sbits += rebalance - (3 << BITRES);

            /* For a stereo split, the high bits of fill are always zero, so no
               folding will be done to the side. */
            cm |= quant_band(ctx, Y, N, sbits, B, NULL, LM, NULL, side, NULL, fill >> B);
        }
        else {
            /* For a stereo split, the high bits of fill are always zero, so no
               folding will be done to the side. */
            cm = quant_band(ctx, Y, N, sbits, B, NULL, LM, NULL, side, NULL, fill >> B);
            rebalance = sbits - (rebalance - ctx->remaining_bits);
            if (rebalance > 3 << BITRES && itheta != 16384)
                mbits += rebalance - (3 << BITRES);
            /* In stereo mode, we do not apply a scaling to the mid because we need the normalized
               mid for folding later. */
            cm |= quant_band(ctx, X, N, mbits, B, lowband, LM, lowband_out, 32767,
                             lowband_scratch, fill);
        }
    }

    /* This code is used by the decoder and by the resynthesis-enabled encoder */
    if (ctx->resynth) {
        if (N != 2)
            stereo_merge(X, Y, mid, N, ctx->arch);
        if (inv)
        {
            int j;
            for (j = 0; j < N; j++)
                Y[j] = -Y[j];
        }
    }
    return cm;
}
//----------------------------------------------------------------------------------------------------------------------

static void special_hybrid_folding(const CELTMode *m, int16_t *norm, int16_t *norm2, int start, int M, int dual_stereo){
    int n1, n2;
    const int16_t *__restrict__ eBands = m->eBands;
    n1 = M * (eBands[start + 1] - eBands[start]);
    n2 = M * (eBands[start + 2] - eBands[start + 1]);
    /* Duplicate enough of the first band folding data to be able to fold the second band.
       Copies no data for CELT-only mode. */
    OPUS_COPY(&norm[n1], &norm[2 * n1 - n2], n2 - n1);
    if (dual_stereo)
        OPUS_COPY(&norm2[n1], &norm2[2 * n1 - n2], n2 - n1);
}
//----------------------------------------------------------------------------------------------------------------------

void quant_all_bands(int encode, const CELTMode *m, int start, int end, int16_t *X_, int16_t *Y_,
                     unsigned char *collapse_masks, const int32_t *bandE, int *pulses, int shortBlocks, int spread,
                     int dual_stereo, int intensity, int *tf_res, int32_t total_bits, int32_t balance, ec_ctx *ec,
                     int LM, int codedBands, uint32_t *seed, int complexity, int arch, int disable_inv){
    int i;
    int32_t remaining_bits;
    const int16_t *__restrict__ eBands = m->eBands;
    int16_t *__restrict__ norm, *__restrict__ norm2;
    VARDECL(int16_t, _norm);
    VARDECL(int16_t, _lowband_scratch);
    VARDECL(int16_t, X_save);
    VARDECL(int16_t, Y_save);
    VARDECL(int16_t, X_save2);
    VARDECL(int16_t, Y_save2);
    VARDECL(int16_t, norm_save2);
    int resynth_alloc;
    int16_t *lowband_scratch;
    int B;
    int M;
    int lowband_offset;
    int update_lowband = 1;
    int C = Y_ != NULL ? 2 : 1;
    int norm_offset;
    int theta_rdo = encode && Y_ != NULL && !dual_stereo && complexity >= 8;

    int resynth = !encode || theta_rdo;

    struct band_ctx ctx;
    SAVE_STACK;

    M = 1 << LM;
    B = shortBlocks ? M : 1;
    norm_offset = M * eBands[start];
    /* No need to allocate norm for the last band because we don't need an
       output in that band. */
    ALLOC(_norm, C * (M * eBands[m->nbEBands - 1] - norm_offset), int16_t);
    norm = _norm;
    norm2 = norm + M * eBands[m->nbEBands - 1] - norm_offset;

    /* For decoding, we can use the last band as scratch space because we don't need that
       scratch space for the last band and we don't care about the data there until we're
       decoding the last band. */
    if (encode && resynth)
        resynth_alloc = M * (eBands[m->nbEBands] - eBands[m->nbEBands - 1]);
    else
        resynth_alloc = ALLOC_NONE;
    ALLOC(_lowband_scratch, resynth_alloc, int16_t);
    if (encode && resynth)
        lowband_scratch = _lowband_scratch;
    else
        lowband_scratch = X_ + M * eBands[m->nbEBands - 1];
    ALLOC(X_save, resynth_alloc, int16_t);
    ALLOC(Y_save, resynth_alloc, int16_t);
    ALLOC(X_save2, resynth_alloc, int16_t);
    ALLOC(Y_save2, resynth_alloc, int16_t);
    ALLOC(norm_save2, resynth_alloc, int16_t);

    lowband_offset = 0;
    ctx.bandE = bandE;
    ctx.ec = ec;
    ctx.encode = encode;
    ctx.intensity = intensity;
    ctx.m = m;
    ctx.seed = *seed;
    ctx.spread = spread;
    ctx.arch = arch;
    ctx.disable_inv = disable_inv;
    ctx.resynth = resynth;
    ctx.theta_round = 0;
    /* Avoid injecting noise in the first band on transients. */
    ctx.avoid_split_noise = B > 1;
    for (i = start; i < end; i++){
        int32_t tell;
        int b;
        int N;
        int32_t curr_balance;
        int effective_lowband = -1;
        int16_t *__restrict__ X, *__restrict__ Y;
        int tf_change = 0;
        unsigned x_cm;
        unsigned y_cm;
        int last;

        ctx.i = i;
        last = (i == end - 1);

        X = X_ + M * eBands[i];
        if (Y_ != NULL)
            Y = Y_ + M * eBands[i];
        else
            Y = NULL;
        N = M * eBands[i + 1] - M * eBands[i];
        celt_assert(N > 0);
        tell = ec_tell_frac(ec);

        /* Compute how many bits we want to allocate to this band */
        if (i != start)
            balance -= tell;
        remaining_bits = total_bits - tell - 1;
        ctx.remaining_bits = remaining_bits;
        if (i <= codedBands - 1){
            curr_balance = celt_sudiv(balance, IMIN(3, codedBands - i));
            b = IMAX(0, IMIN(16383, IMIN(remaining_bits + 1, pulses[i] + curr_balance)));
        }
        else {
            b = 0;
        }

        if (resynth && (M * eBands[i] - N >= M * eBands[start] || i == start + 1) && (update_lowband || lowband_offset == 0))
            lowband_offset = i;
        if (i == start + 1)
            special_hybrid_folding(m, norm, norm2, start, M, dual_stereo);

        tf_change = tf_res[i];
        ctx.tf_change = tf_change;
        if (i >= m->effEBands) {
            X = norm;
            if (Y_ != NULL)
                Y = norm;
            lowband_scratch = NULL;
        }
        if (last && !theta_rdo)
            lowband_scratch = NULL;

        /* Get a conservative estimate of the collapse_mask's for the bands we're
           going to be folding from. */
        if (lowband_offset != 0 && (spread != SPREAD_AGGRESSIVE || B > 1 || tf_change < 0)) {
            int fold_start;
            int fold_end;
            int fold_i;
            /* This ensures we never repeat spectral content within one band */
            effective_lowband = IMAX(0, M * eBands[lowband_offset] - norm_offset - N);
            fold_start = lowband_offset;
            while (M * eBands[--fold_start] > effective_lowband + norm_offset)
                ;
            fold_end = lowband_offset - 1;

            while (++fold_end < i && M * eBands[fold_end] < effective_lowband + norm_offset + N)
                ;

            x_cm = y_cm = 0;
            fold_i = fold_start;
            do {
                x_cm |= collapse_masks[fold_i * C + 0];
                y_cm |= collapse_masks[fold_i * C + C - 1];
            } while (++fold_i < fold_end);
        }
        /* Otherwise, we'll be using the LCG to fold, so all blocks will (almost
           always) be non-zero. */
        else
            x_cm = y_cm = (1 << B) - 1;

        if (dual_stereo && i == intensity) {
            int j;

            /* Switch off dual stereo to do intensity. */
            dual_stereo = 0;
            if (resynth)
                for (j = 0; j < M * eBands[i] - norm_offset; j++)
                    norm[j] = HALF32(norm[j] + norm2[j]);
        }
        if (dual_stereo) {
            x_cm = quant_band(&ctx, X, N, b / 2, B,
                              effective_lowband != -1 ? norm + effective_lowband : NULL, LM,
                              last ? NULL : norm + M * eBands[i] - norm_offset, 32767, lowband_scratch, x_cm);
            y_cm = quant_band(&ctx, Y, N, b / 2, B,
                              effective_lowband != -1 ? norm2 + effective_lowband : NULL, LM,
                              last ? NULL : norm2 + M * eBands[i] - norm_offset, 32767, lowband_scratch, y_cm);
        }
        else {
            if (Y != NULL) {
                if (theta_rdo && i < intensity) {
                    ec_ctx ec_save, ec_save2;
                    struct band_ctx ctx_save, ctx_save2;
                    int32_t dist0, dist1;
                    unsigned cm, cm2;
                    int nstart_bytes, nend_bytes, save_bytes;
                    unsigned char *bytes_buf;
                    unsigned char bytes_save[1275];
                    int16_t w[2];
                    compute_channel_weights(bandE[i], bandE[i + m->nbEBands], w);
                    /* Make a copy. */
                    cm = x_cm | y_cm;
                    ec_save = *ec;
                    ctx_save = ctx;
                    OPUS_COPY(X_save, X, N);
                    OPUS_COPY(Y_save, Y, N);
                    /* Encode and round down. */
                    ctx.theta_round = -1;
                    x_cm = quant_band_stereo(&ctx, X, Y, N, b, B,
                                             effective_lowband != -1 ? norm + effective_lowband : NULL, LM,
                                             last ? NULL : norm + M * eBands[i] - norm_offset, lowband_scratch, cm);

                    dist0 = MULT16_32_Q15(w[0], celt_inner_prod(X_save, X, N, arch)) +
                            MULT16_32_Q15(w[1], celt_inner_prod(Y_save, Y, N, arch));

                    /* Save first result. */
                    cm2 = x_cm;
                    ec_save2 = *ec;
                    ctx_save2 = ctx;
                    OPUS_COPY(X_save2, X, N);
                    OPUS_COPY(Y_save2, Y, N);
                    if (!last)
                        OPUS_COPY(norm_save2, norm + M * eBands[i] - norm_offset, N);
                    nstart_bytes = ec_save.offs;
                    nend_bytes = ec_save.storage;
                    bytes_buf = ec_save.buf + nstart_bytes;
                    save_bytes = nend_bytes - nstart_bytes;
                    OPUS_COPY(bytes_save, bytes_buf, save_bytes);

                    /* Restore */
                    *ec = ec_save;
                    ctx = ctx_save;
                    OPUS_COPY(X, X_save, N);
                    OPUS_COPY(Y, Y_save, N);

                    if (i == start + 1)
                        special_hybrid_folding(m, norm, norm2, start, M, dual_stereo);

                    /* Encode and round up. */
                    ctx.theta_round = 1;
                    x_cm = quant_band_stereo(&ctx, X, Y, N, b, B,
                                             effective_lowband != -1 ? norm + effective_lowband : NULL, LM,
                                             last ? NULL : norm + M * eBands[i] - norm_offset, lowband_scratch, cm);
                    dist1 = MULT16_32_Q15(w[0], celt_inner_prod(X_save, X, N, arch)) + MULT16_32_Q15(w[1], celt_inner_prod(Y_save, Y, N, arch));
                    if (dist0 >= dist1) {
                        x_cm = cm2;
                        *ec = ec_save2;
                        ctx = ctx_save2;
                        OPUS_COPY(X, X_save2, N);
                        OPUS_COPY(Y, Y_save2, N);
                        if (!last)
                            OPUS_COPY(norm + M * eBands[i] - norm_offset, norm_save2, N);
                        OPUS_COPY(bytes_buf, bytes_save, save_bytes);
                    }
                }
                else {
                    ctx.theta_round = 0;
                    x_cm = quant_band_stereo(&ctx, X, Y, N, b, B,
                                             effective_lowband != -1 ? norm + effective_lowband : NULL, LM,
                                             last ? NULL : norm + M * eBands[i] - norm_offset, lowband_scratch, x_cm | y_cm);
                }
            }
            else {
                x_cm = quant_band(&ctx, X, N, b, B,
                                  effective_lowband != -1 ? norm + effective_lowband : NULL, LM,
                                  last ? NULL : norm + M * eBands[i] - norm_offset, 32767, lowband_scratch, x_cm | y_cm);
            }
            y_cm = x_cm;
        }
        collapse_masks[i * C + 0] = (unsigned char)x_cm;
        collapse_masks[i * C + C - 1] = (unsigned char)y_cm;
        balance += pulses[i] + tell;

        /* Update the folding position only as long as we have 1 bit/sample depth. */
        update_lowband = b > (N << BITRES);
        /* We only need to avoid noise on a split for the first band. After that, we
           have folding. */
        ctx.avoid_split_noise = 0;
    }
    *seed = ctx.seed;
}
//----------------------------------------------------------------------------------------------------------------------

int opus_custom_decoder_get_size(const CELTMode *mode, int channels){
    static int size;
    size = sizeof(struct CELTDecoder) + (channels * (DECODE_BUFFER_SIZE + mode->overlap) - 1) * sizeof(int32_t) + channels * LPC_ORDER * sizeof(int16_t) + 4 * 2 * mode->nbEBands * sizeof(int16_t);
    return size;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_decoder_get_size(int channels){
    const CELTMode *mode = opus_custom_mode_create(48000, 960, NULL);
    return opus_custom_decoder_get_size(mode, channels);
}
//----------------------------------------------------------------------------------------------------------------------

int opus_custom_decoder_init(CELTDecoder *st, const CELTMode *mode, int channels){
    if (channels < 0 || channels > 2)
        return OPUS_BAD_ARG;

    if (st == NULL)
        return OPUS_ALLOC_FAIL;

    OPUS_CLEAR((char *)st, opus_custom_decoder_get_size(mode, channels));

    st->mode = mode;
    st->overlap = mode->overlap;
    st->stream_channels = st->channels = channels;

    st->downsample = 1;
    st->start = 0;
    st->end = st->mode->effEBands;
    st->signalling = 1;

    st->disable_inv = channels == 1;

    st->arch = opus_select_arch();

    celt_decoder_ctl(st, OPUS_RESET_STATE);

    return OPUS_OK;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_decoder_init(CELTDecoder *st, int32_t sampling_rate, int channels){
    int ret;
    ret = opus_custom_decoder_init(st, opus_custom_mode_create(48000, 960, NULL), channels);
    if (ret != OPUS_OK)
        return ret;
    st->downsample = resampling_factor(sampling_rate);
    if (st->downsample == 0)
        return OPUS_BAD_ARG;
    else
        return OPUS_OK;
}
//----------------------------------------------------------------------------------------------------------------------

/* Special case for stereo with no downsampling and no accumulation. This is quite common and we can make it faster by
   processing both channels in the same loop, reducing overhead due to the dependency loop in the IIR filter. */
static void deemphasis_stereo_simple(int32_t *in[], int16_t *pcm, int N, const int16_t coef0, int32_t *mem) {
    int32_t *__restrict__ x0;
    int32_t *__restrict__ x1;
    int32_t m0, m1;
    int j;
    x0 = in[0];
    x1 = in[1];
    m0 = mem[0];
    m1 = mem[1];
    for (j = 0; j < N; j++) {
        int32_t tmp0, tmp1;
        /* Add VERY_SMALL to x[] first to reduce dependency chain. */
        tmp0 = x0[j] + VERY_SMALL + m0;
        tmp1 = x1[j] + VERY_SMALL + m1;
        m0 = MULT16_32_Q15(coef0, tmp0);
        m1 = MULT16_32_Q15(coef0, tmp1);
        pcm[2 * j] = SCALEOUT(sig2word16(tmp0));
        pcm[2 * j + 1] = SCALEOUT(sig2word16(tmp1));
    }
    mem[0] = m0;
    mem[1] = m1;
}
//----------------------------------------------------------------------------------------------------------------------

static void deemphasis(int32_t *in[], int16_t *pcm, int N, int C, int downsample, const int16_t *coef,
               int32_t *mem, int accum) {
    int c;
    int Nd;
    int apply_downsampling = 0;
    int16_t coef0;
    VARDECL(int32_t, scratch);
    SAVE_STACK;

    /* Short version for common case. */
    if (downsample == 1 && C == 2 && !accum) {
        deemphasis_stereo_simple(in, pcm, N, coef[0], mem);
        return;
    }

    ALLOC(scratch, N, int32_t);
    coef0 = coef[0];
    Nd = N / downsample;
    c = 0;
    do  {
        int j;
        int32_t *__restrict__ x;
        int16_t *__restrict__ y;
        int32_t m = mem[c];
        x = in[c];
        y = pcm + c;

        if (downsample > 1) {
            /* Shortcut for the standard (non-custom modes) case */
            for (j = 0; j < N; j++) {
                int32_t tmp = x[j] + VERY_SMALL + m;
                m = MULT16_32_Q15(coef0, tmp);
                scratch[j] = tmp;
            }
            apply_downsampling = 1;
        }
        else {
            /* Shortcut for the standard (non-custom modes) case */

            if (accum) {
                for (j = 0; j < N; j++) {
                    int32_t tmp = x[j] + m + VERY_SMALL;
                    m = MULT16_32_Q15(coef0, tmp);
                    y[j * C] = SAT16(ADD32(y[j * C], SCALEOUT(sig2word16(tmp))));
                }
            }
            else {
                for (j = 0; j < N; j++) {
                    int32_t tmp = x[j] + VERY_SMALL + m;
                    m = MULT16_32_Q15(coef0, tmp);
                    y[j * C] = SCALEOUT(sig2word16(tmp));
                }
            }
        }
        mem[c] = m;

        if (apply_downsampling) {
            /* Perform down-sampling */

            if (accum)  {
                for (j = 0; j < Nd; j++)
                    y[j * C] = SAT16(ADD32(y[j * C], SCALEOUT(sig2word16(scratch[j * downsample]))));
            }
            else {
                for (j = 0; j < Nd; j++)
                    y[j * C] = SCALEOUT(sig2word16(scratch[j * downsample]));
            }
        }
    } while (++c < C);
}
//----------------------------------------------------------------------------------------------------------------------

static void celt_synthesis(const CELTMode *mode, int16_t *X, int32_t *out_syn[], int16_t *oldBandE, int start,
                           int effEnd, int C, int CC, int isTransient, int LM, int downsample, int silence, int arch){
    int c, i;
    int M;
    int b;
    int B;
    int N, NB;
    int shift;
    int nbEBands;
    int overlap;
    VARDECL(int32_t, freq);
    SAVE_STACK;

    overlap = mode->overlap;
    nbEBands = mode->nbEBands;
    N = mode->shortMdctSize << LM;
    ALLOC(freq, N, int32_t); /**< Interleaved signal MDCTs */
    M = 1 << LM;

    if (isTransient) {
        B = M;
        NB = mode->shortMdctSize;
        shift = mode->maxLM;
    }
    else {
        B = 1;
        NB = mode->shortMdctSize << LM;
        shift = mode->maxLM - LM;
    }

    if (CC == 2 && C == 1) {
        /* Copying a mono streams to two channels */
        int32_t *freq2;
        denormalise_bands(mode, X, freq, oldBandE, start, effEnd, M,
                          downsample, silence);
        /* Store a temporary copy in the output buffer because the IMDCT destroys its input. */
        freq2 = out_syn[1] + overlap / 2;
        OPUS_COPY(freq2, freq, N);
        for (b = 0; b < B; b++)
            clt_mdct_backward(&mode->mdct, &freq2[b], out_syn[0] + NB * b, mode->window, overlap, shift, B, arch);
        for (b = 0; b < B; b++)
            clt_mdct_backward(&mode->mdct, &freq[b], out_syn[1] + NB * b, mode->window, overlap, shift, B, arch);
    }
    else if (CC == 1 && C == 2) {
        /* Downmixing a stereo stream to mono */
        int32_t *freq2;
        freq2 = out_syn[0] + overlap / 2;
        denormalise_bands(mode, X, freq, oldBandE, start, effEnd, M,
                          downsample, silence);
        /* Use the output buffer as temp array before downmixing. */
        denormalise_bands(mode, X + N, freq2, oldBandE + nbEBands, start, effEnd, M,
                          downsample, silence);
        for (i = 0; i < N; i++)
            freq[i] = ADD32(HALF32(freq[i]), HALF32(freq2[i]));
        for (b = 0; b < B; b++)
            clt_mdct_backward(&mode->mdct, &freq[b], out_syn[0] + NB * b, mode->window, overlap, shift, B, arch);
    }
    else {
        /* Normal case (mono or stereo) */
        c = 0;
        do {
            denormalise_bands(mode, X + c * N, freq, oldBandE + c * nbEBands, start, effEnd, M,
                              downsample, silence);
            for (b = 0; b < B; b++)
                clt_mdct_backward(&mode->mdct, &freq[b], out_syn[c] + NB * b, mode->window, overlap, shift, B, arch);
        } while (++c < CC);
    }
    /* Saturate IMDCT output so that we can't overflow in the pitch postfilter
       or in the */
    c = 0;
    do {
        for (i = 0; i < N; i++)
            out_syn[c][i] = SATURATE(out_syn[c][i], SIG_SAT);
    } while (++c < CC);
}
//----------------------------------------------------------------------------------------------------------------------

static void tf_decode(int start, int end, int isTransient, int *tf_res, int LM, ec_dec *dec){
    int i, curr, tf_select;
    int tf_select_rsv;
    int tf_changed;
    int logp;
    uint32_t budget;
    uint32_t tell;

    budget = dec->storage * 8;
    tell = ec_tell(dec);
    logp = isTransient ? 2 : 4;
    tf_select_rsv = LM > 0 && tell + logp + 1 <= budget;
    budget -= tf_select_rsv;
    tf_changed = curr = 0;
    for (i = start; i < end; i++) {
        if (tell + logp <= budget) {
            curr ^= ec_dec_bit_logp(dec, logp);
            tell = ec_tell(dec);
            tf_changed |= curr;
        }
        tf_res[i] = curr;
        logp = isTransient ? 4 : 5;
    }
    tf_select = 0;
    if (tf_select_rsv &&
        tf_select_table[LM][4 * isTransient + 0 + tf_changed] !=
            tf_select_table[LM][4 * isTransient + 2 + tf_changed]) {
        tf_select = ec_dec_bit_logp(dec, 1);
    }
    for (i = start; i < end; i++) {
        tf_res[i] = tf_select_table[LM][4 * isTransient + 2 * tf_select + tf_res[i]];
    }
}
//----------------------------------------------------------------------------------------------------------------------

static int celt_plc_pitch_search(int32_t *decode_mem[2], int C, int arch) {
    int pitch_index;
    VARDECL(int16_t, lp_pitch_buf);
    SAVE_STACK;
    int16_t *lp_pitch_buf = (int16_t *)malloc((DECODE_BUFFER_SIZE >> 1) * sizeof(int16_t));
                                                        // ALLOC( lp_pitch_buf, DECODE_BUFFER_SIZE>>1, int16_t );
    pitch_downsample(decode_mem, lp_pitch_buf,
                     DECODE_BUFFER_SIZE, C, arch);
    pitch_search(lp_pitch_buf + (PLC_PITCH_LAG_MAX >> 1), lp_pitch_buf,
                 DECODE_BUFFER_SIZE - PLC_PITCH_LAG_MAX,
                 PLC_PITCH_LAG_MAX - PLC_PITCH_LAG_MIN, &pitch_index, arch);
    pitch_index = PLC_PITCH_LAG_MAX - pitch_index;

    free(lp_pitch_buf);
    return pitch_index;
}
//----------------------------------------------------------------------------------------------------------------------

static void celt_decode_lost(CELTDecoder *__restrict__ st, int N, int LM){
    int c;
    int i;
    const int C = st->channels;
    int32_t *decode_mem[2];
    int32_t *out_syn[2];
    int16_t *lpc;
    int16_t *oldBandE, *oldLogE, *oldLogE2, *backgroundLogE;
    const CELTMode *mode;
    int nbEBands;
    int overlap;
    int start;
    int loss_count;
    int noise_based;
    const int16_t *eBands;
    SAVE_STACK;

    mode = st->mode;
    nbEBands = mode->nbEBands;
    overlap = mode->overlap;
    eBands = mode->eBands;

    c = 0;
    do {
        decode_mem[c] = st->_decode_mem + c * (DECODE_BUFFER_SIZE + overlap);
        out_syn[c] = decode_mem[c] + DECODE_BUFFER_SIZE - N;
    } while (++c < C);
    lpc = (int16_t *)(st->_decode_mem + (DECODE_BUFFER_SIZE + overlap) * C);
    oldBandE = lpc + C * LPC_ORDER;
    oldLogE = oldBandE + 2 * nbEBands;
    oldLogE2 = oldLogE + 2 * nbEBands;
    backgroundLogE = oldLogE2 + 2 * nbEBands;

    loss_count = st->loss_count;
    start = st->start;
    noise_based = loss_count >= 5 || start != 0 || st->skip_plc;
    if (noise_based){
        /* Noise-based PLC/CNG */

        VARDECL(int16_t, X);

        uint32_t seed;
        int end;
        int effEnd;
        int16_t decay;
        end = st->end;
        effEnd = IMAX(start, IMIN(end, mode->effEBands));

        ALLOC(X, C * N, int16_t); /**< Interleaved normalised MDCTs */

        /* Energy decay */
        decay = loss_count == 0 ? QCONST16(1.5f, DB_SHIFT) : QCONST16(.5f, DB_SHIFT);
        c = 0;
        do {
            for (i = start; i < end; i++)
                oldBandE[c * nbEBands + i] = MAX16(backgroundLogE[c * nbEBands + i], oldBandE[c * nbEBands + i] - decay);
        } while (++c < C);
        seed = st->rng;
        for (c = 0; c < C; c++) {
            for (i = start; i < effEnd; i++)
            {
                int j;
                int boffs;
                int blen;
                boffs = N * c + (eBands[i] << LM);
                blen = (eBands[i + 1] - eBands[i]) << LM;
                for (j = 0; j < blen; j++)
                {
                    seed = celt_lcg_rand(seed);
                    X[boffs + j] = (int16_t)((int32_t)seed >> 20);
                }
                renormalise_vector(X + boffs, blen, 32767, st->arch);
            }
        }
        st->rng = seed;

        c = 0;
        do {
            OPUS_MOVE(decode_mem[c], decode_mem[c] + N,
                      DECODE_BUFFER_SIZE - N + (overlap >> 1));
        } while (++c < C);

        celt_synthesis(mode, X, out_syn, oldBandE, start, effEnd, C, C, 0, LM, st->downsample, 0, st->arch);
    }
    else{
        int exc_length;
        /* Pitch-based PLC */
        const int16_t *window;
        int16_t *exc;
        int16_t fade = 32767;
        int pitch_index;
        VARDECL(int32_t, etmp);
        VARDECL(int16_t, _exc);
        VARDECL(int16_t, fir_tmp);

        if (loss_count == 0) {
            st->last_pitch_index = pitch_index = celt_plc_pitch_search(decode_mem, C, st->arch);
        }
        else {
            pitch_index = st->last_pitch_index;
            fade = QCONST16(.8f, 15);
        }

        /* We want the excitation for 2 pitch periods in order to look for a
           decaying signal, but we can't get more than MAX_PERIOD. */
        exc_length = IMIN(2 * pitch_index, MAX_PERIOD);

        ALLOC(etmp, overlap, int32_t);
        ALLOC(_exc, MAX_PERIOD + LPC_ORDER, int16_t);
        ALLOC(fir_tmp, exc_length, int16_t);
        exc = _exc + LPC_ORDER;
        window = mode->window;
        c = 0;
        do {
            int16_t decay;
            int16_t attenuation;
            int32_t S1 = 0;
            int32_t *buf;
            int extrapolation_offset;
            int extrapolation_len;
            int j;

            buf = decode_mem[c];
            for (i = 0; i < MAX_PERIOD + LPC_ORDER; i++)
                exc[i - LPC_ORDER] = ROUND16(buf[DECODE_BUFFER_SIZE - MAX_PERIOD - LPC_ORDER + i], 12);

            if (loss_count == 0) {
                int32_t ac[LPC_ORDER + 1];
                /* Compute LPC coefficients for the last MAX_PERIOD samples before
                   the first loss so we can work in the excitation-filter domain. */
                _celt_autocorr(exc, ac, window, overlap,
                               LPC_ORDER, MAX_PERIOD, st->arch);
                /* Add a noise floor of -40 dB. */

                ac[0] += SHR32(ac[0], 13);

                /* Use lag windowing to stabilize the Levinson-Durbin recursion. */
                for (i = 1; i <= LPC_ORDER; i++) {
                    /*ac[i] *= exp(-.5*(2*M_PI*.002*i)*(2*M_PI*.002*i));*/

                    ac[i] -= MULT16_32_Q15(2 * i * i, ac[i]);
                }
                _celt_lpc(lpc + c * LPC_ORDER, ac, LPC_ORDER);

                /* For fixed-point, apply bandwidth expansion until we can guarantee that
                   no overflow can happen in the IIR filter. This means:
                   32768*sum(abs(filter)) < 2^31 */
                while (1)  {
                    int16_t tmp = 32767;
                    int32_t sum = QCONST16(1., 12);
                    for (i = 0; i < LPC_ORDER; i++)
                        sum += ABS16(lpc[c * LPC_ORDER + i]);
                    if (sum < 65535)
                        break;
                    for (i = 0; i < LPC_ORDER; i++)
                    {
                        tmp = MULT16_16_Q15(QCONST16(.99f, 15), tmp);
                        lpc[c * LPC_ORDER + i] = MULT16_16_Q15(lpc[c * LPC_ORDER + i], tmp);
                    }
                }
            }
            /* Initialize the LPC history with the samples just before the start
               of the region for which we're computing the excitation. */
            {
                /* Compute the excitation for exc_length samples before the loss. We need the copy
                   because celt_fir() cannot filter in-place. */
                celt_fir(exc + MAX_PERIOD - exc_length, lpc + c * LPC_ORDER,
                         fir_tmp, exc_length, LPC_ORDER, st->arch);
                OPUS_COPY(exc + MAX_PERIOD - exc_length, fir_tmp, exc_length);
            }

            /* Check if the waveform is decaying, and if so how fast.
               We do this to avoid adding energy when concealing in a segment
               with decaying energy. */
            {
                int32_t E1 = 1, E2 = 1;
                int decay_length;

                int shift = IMAX(0, 2 * celt_zlog2(celt_maxabs16(&exc[MAX_PERIOD - exc_length], exc_length)) - 20);

                decay_length = exc_length >> 1;
                for (i = 0; i < decay_length; i++) {
                    int16_t e;
                    e = exc[MAX_PERIOD - decay_length + i];
                    E1 += SHR32(MULT16_16(e, e), shift);
                    e = exc[MAX_PERIOD - 2 * decay_length + i];
                    E2 += SHR32(MULT16_16(e, e), shift);
                }
                E1 = MIN32(E1, E2);
                decay = celt_sqrt(frac_div32(SHR32(E1, 1), E2));
            }

            /* Move the decoder memory one frame to the left to give us room to
               add the data for the new frame. We ignore the overlap that extends
               past the end of the buffer, because we aren't going to use it. */
            OPUS_MOVE(buf, buf + N, DECODE_BUFFER_SIZE - N);

            /* Extrapolate from the end of the excitation with a period of
               "pitch_index", scaling down each period by an additional factor of
               "decay". */
            extrapolation_offset = MAX_PERIOD - pitch_index;
            /* We need to extrapolate enough samples to cover a complete MDCT
               window (including overlap/2 samples on both sides). */
            extrapolation_len = N + overlap;
            /* We also apply fading if this is not the first loss. */
            attenuation = MULT16_16_Q15(fade, decay);
            for (i = j = 0; i < extrapolation_len; i++, j++) {
                int16_t tmp;
                if (j >= pitch_index) {
                    j -= pitch_index;
                    attenuation = MULT16_16_Q15(attenuation, decay);
                }
                buf[DECODE_BUFFER_SIZE - N + i] =
                    SHL32(EXTEND32(MULT16_16_Q15(attenuation,
                                                 exc[extrapolation_offset + j])),
                          12);
                /* Compute the energy of the previously decoded signal whose
                   excitation we're copying. */
                tmp = ROUND16(
                    buf[DECODE_BUFFER_SIZE - MAX_PERIOD - N + extrapolation_offset + j],
                    12);
                S1 += SHR32(MULT16_16(tmp, tmp), 10);
            }
            {
                int16_t lpc_mem[LPC_ORDER];
                /* Copy the last decoded samples (prior to the overlap region) to
                   synthesis filter memory so we can have a continuous signal. */
                for (i = 0; i < LPC_ORDER; i++)
                    lpc_mem[i] = ROUND16(buf[DECODE_BUFFER_SIZE - N - 1 - i], 12);
                /* Apply the synthesis filter to convert the excitation back into
                   the signal domain. */
                celt_iir(buf + DECODE_BUFFER_SIZE - N, lpc + c * LPC_ORDER,
                         buf + DECODE_BUFFER_SIZE - N, extrapolation_len, LPC_ORDER,
                         lpc_mem, st->arch);

                for (i = 0; i < extrapolation_len; i++)
                    buf[DECODE_BUFFER_SIZE - N + i] = SATURATE(buf[DECODE_BUFFER_SIZE - N + i], SIG_SAT);
            }

            /* Check if the synthesis energy is higher than expected, which can
               happen with the signal changes during our window. If so,
               attenuate. */
            {
                int32_t S2 = 0;
                for (i = 0; i < extrapolation_len; i++)
                {
                    int16_t tmp = ROUND16(buf[DECODE_BUFFER_SIZE - N + i], 12);
                    S2 += SHR32(MULT16_16(tmp, tmp), 10);
                }
                /* This checks for an "explosion" in the synthesis. */

                if (!(S1 > SHR32(S2, 2))) {
                    for (i = 0; i < extrapolation_len; i++)
                        buf[DECODE_BUFFER_SIZE - N + i] = 0;
                }
                else if (S1 < S2) {
                    int16_t ratio = celt_sqrt(frac_div32(SHR32(S1, 1) + 1, S2 + 1));
                    for (i = 0; i < overlap; i++) {
                        int16_t tmp_g = 32767 - MULT16_16_Q15(window[i], 32767 - ratio);
                        buf[DECODE_BUFFER_SIZE - N + i] =
                            MULT16_32_Q15(tmp_g, buf[DECODE_BUFFER_SIZE - N + i]);
                    }
                    for (i = overlap; i < extrapolation_len; i++) {
                        buf[DECODE_BUFFER_SIZE - N + i] =
                            MULT16_32_Q15(ratio, buf[DECODE_BUFFER_SIZE - N + i]);
                    }
                }
            }

            /* Apply the pre-filter to the MDCT overlap for the next frame because
               the post-filter will be re-applied in the decoder after the MDCT
               overlap. */
            comb_filter(etmp, buf + DECODE_BUFFER_SIZE,
                        st->postfilter_period, st->postfilter_period, overlap,
                        -st->postfilter_gain, -st->postfilter_gain,
                        st->postfilter_tapset, st->postfilter_tapset, NULL, 0, st->arch);

            /* Simulate TDAC on the concealed audio so that it blends with the
               MDCT of the next frame. */
            for (i = 0; i < overlap / 2; i++) {
                buf[DECODE_BUFFER_SIZE + i] =
                    MULT16_32_Q15(window[i], etmp[overlap - 1 - i]) + MULT16_32_Q15(window[overlap - i - 1], etmp[i]);
            }
        } while (++c < C);
    }
    st->loss_count = loss_count + 1;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_decode_with_ec(CELTDecoder *__restrict__ st, const unsigned char *data, int len, int16_t *__restrict__ pcm,
                        int frame_size, ec_dec *dec, int accum) {
    int c, i, N;
    int spread_decision;
    int32_t bits;
    ec_dec _dec;

    VARDECL(int16_t, X);

    VARDECL(int, fine_quant);
    VARDECL(int, pulses);
    VARDECL(int, cap);
    VARDECL(int, offsets);
    VARDECL(int, fine_priority);
    VARDECL(int, tf_res);
    VARDECL(unsigned char, collapse_masks);
    int32_t *decode_mem[2];
    int32_t *out_syn[2];
    int16_t *lpc;
    int16_t *oldBandE, *oldLogE, *oldLogE2, *backgroundLogE;

    int shortBlocks;
    int isTransient;
    int intra_ener;
    const int CC = st->channels;
    int LM, M;
    int start;
    int end;
    int effEnd;
    int codedBands;
    int alloc_trim;
    int postfilter_pitch;
    int16_t postfilter_gain;
    int intensity = 0;
    int dual_stereo = 0;
    int32_t total_bits;
    int32_t balance;
    int32_t tell;
    int dynalloc_logp;
    int postfilter_tapset;
    int anti_collapse_rsv;
    int anti_collapse_on = 0;
    int silence;
    int C = st->stream_channels;
    const CELTMode *mode;
    int nbEBands;
    int overlap;
    const int16_t *eBands;
    ALLOC_STACK;

    VALIDATE_CELT_DECODER(st);
    mode = st->mode;
    nbEBands = mode->nbEBands;
    overlap = mode->overlap;
    eBands = mode->eBands;
    start = st->start;
    end = st->end;
    frame_size *= st->downsample;

    lpc = (int16_t *)(st->_decode_mem + (DECODE_BUFFER_SIZE + overlap) * CC);
    oldBandE = lpc + CC * LPC_ORDER;
    oldLogE = oldBandE + 2 * nbEBands;
    oldLogE2 = oldLogE + 2 * nbEBands;
    backgroundLogE = oldLogE2 + 2 * nbEBands;

    {
        for (LM = 0; LM <= mode->maxLM; LM++)
            if (mode->shortMdctSize << LM == frame_size)
                break;
        if (LM > mode->maxLM)
            return OPUS_BAD_ARG;
    }

    M = 1 << LM;

    if (len < 0 || len > 1275 || pcm == NULL)
        return OPUS_BAD_ARG;

    N = M * mode->shortMdctSize;
    c = 0;
    do {
        decode_mem[c] = st->_decode_mem + c * (DECODE_BUFFER_SIZE + overlap);
        out_syn[c] = decode_mem[c] + DECODE_BUFFER_SIZE - N;
    } while (++c < CC);

    effEnd = end;
    if (effEnd > mode->effEBands)
        effEnd = mode->effEBands;

    if (data == NULL || len <= 1) {
        celt_decode_lost(st, N, LM);
        deemphasis(out_syn, pcm, N, CC, st->downsample, mode->preemph, st->preemph_memD, accum);

        return frame_size / st->downsample;
    }

    /* Check if there are at least two packets received consecutively before
     * turning on the pitch-based PLC */
    st->skip_plc = st->loss_count != 0;

    if (dec == NULL) {
        ec_dec_init(&_dec, (unsigned char *)data, len);
        dec = &_dec;
    }

    if (C == 1) {
        for (i = 0; i < nbEBands; i++)
            oldBandE[i] = MAX16(oldBandE[i], oldBandE[nbEBands + i]);
    }

    total_bits = len * 8;
    tell = ec_tell(dec);

    if (tell >= total_bits)
        silence = 1;
    else if (tell == 1)
        silence = ec_dec_bit_logp(dec, 15);
    else
        silence = 0;
    if (silence)  {
        /* Pretend we've read all the remaining bits */
        tell = len * 8;
        dec->nbits_total += tell - ec_tell(dec);
    }

    postfilter_gain = 0;
    postfilter_pitch = 0;
    postfilter_tapset = 0;
    if (start == 0 && tell + 16 <= total_bits) {
        if (ec_dec_bit_logp(dec, 1))
        {
            int qg, octave;
            octave = ec_dec_uint(dec, 6);
            postfilter_pitch = (16 << octave) + ec_dec_bits(dec, 4 + octave) - 1;
            qg = ec_dec_bits(dec, 3);
            if (ec_tell(dec) + 2 <= total_bits)
                postfilter_tapset = ec_dec_icdf(dec, tapset_icdf, 2);
            postfilter_gain = QCONST16(.09375f, 15) * (qg + 1);
        }
        tell = ec_tell(dec);
    }

    if (LM > 0 && tell + 3 <= total_bits) {
        isTransient = ec_dec_bit_logp(dec, 3);
        tell = ec_tell(dec);
    }
    else
        isTransient = 0;

    if (isTransient)
        shortBlocks = M;
    else
        shortBlocks = 0;

    /* Decode the global flags (first symbols in the stream) */
    intra_ener = tell + 3 <= total_bits ? ec_dec_bit_logp(dec, 3) : 0;
    /* Get band energies */
    unquant_coarse_energy(mode, start, end, oldBandE,
                          intra_ener, dec, C, LM);

    ALLOC(tf_res, nbEBands, int);
    tf_decode(start, end, isTransient, tf_res, LM, dec);

    tell = ec_tell(dec);
    spread_decision = SPREAD_NORMAL;
    if (tell + 4 <= total_bits)
        spread_decision = ec_dec_icdf(dec, spread_icdf, 5);

    ALLOC(cap, nbEBands, int);

    init_caps(mode, cap, LM, C);

    ALLOC(offsets, nbEBands, int);

    dynalloc_logp = 6;
    total_bits <<= BITRES;
    tell = ec_tell_frac(dec);
    for (i = start; i < end; i++) {
        int width, quanta;
        int dynalloc_loop_logp;
        int boost;
        width = C * (eBands[i + 1] - eBands[i]) << LM;
        /* quanta is 6 bits, but no more than 1 bit/sample
           and no less than 1/8 bit/sample */
        quanta = IMIN(width << BITRES, IMAX(6 << BITRES, width));
        dynalloc_loop_logp = dynalloc_logp;
        boost = 0;
        while (tell + (dynalloc_loop_logp << BITRES) < total_bits && boost < cap[i])
        {
            int flag;
            flag = ec_dec_bit_logp(dec, dynalloc_loop_logp);
            tell = ec_tell_frac(dec);
            if (!flag)
                break;
            boost += quanta;
            total_bits -= quanta;
            dynalloc_loop_logp = 1;
        }
        offsets[i] = boost;
        /* Making dynalloc more likely */
        if (boost > 0)
            dynalloc_logp = IMAX(2, dynalloc_logp - 1);
    }

    ALLOC(fine_quant, nbEBands, int);
    alloc_trim = tell + (6 << BITRES) <= total_bits ? ec_dec_icdf(dec, trim_icdf, 7) : 5;

    bits = (((int32_t)len * 8) << BITRES) - ec_tell_frac(dec) - 1;
    anti_collapse_rsv = isTransient && LM >= 2 && bits >= ((LM + 2) << BITRES) ? (1 << BITRES) : 0;
    bits -= anti_collapse_rsv;

    ALLOC(pulses, nbEBands, int);
    ALLOC(fine_priority, nbEBands, int);

    codedBands = clt_compute_allocation(mode, start, end, offsets, cap,
                                        alloc_trim, &intensity, &dual_stereo, bits, &balance, pulses,
                                        fine_quant, fine_priority, C, LM, dec, 0, 0, 0);

    unquant_fine_energy(mode, start, end, oldBandE, fine_quant, dec, C);

    c = 0;
    do {
        OPUS_MOVE(decode_mem[c], decode_mem[c] + N, DECODE_BUFFER_SIZE - N + overlap / 2);
    } while (++c < CC);

    /* Decode fixed codebook */
    ALLOC(collapse_masks, C * nbEBands, unsigned char);

    ALLOC(X, C * N, int16_t); /**< Interleaved normalised MDCTs */

    quant_all_bands(0, mode, start, end, X, C == 2 ? X + N : NULL, collapse_masks,
                    NULL, pulses, shortBlocks, spread_decision, dual_stereo, intensity, tf_res,
                    len * (8 << BITRES) - anti_collapse_rsv, balance, dec, LM, codedBands, &st->rng, 0,
                    st->arch, st->disable_inv);

    if (anti_collapse_rsv > 0) {
        anti_collapse_on = ec_dec_bits(dec, 1);
    }

    unquant_energy_finalise(mode, start, end, oldBandE,
                            fine_quant, fine_priority, len * 8 - ec_tell(dec), dec, C);

    if (anti_collapse_on)
        anti_collapse(mode, X, collapse_masks, LM, C, N,
                      start, end, oldBandE, oldLogE, oldLogE2, pulses, st->rng, st->arch);

    if (silence) {
        for (i = 0; i < C * nbEBands; i++)
            oldBandE[i] = -QCONST16(28.f, DB_SHIFT);
    }

    celt_synthesis(mode, X, out_syn, oldBandE, start, effEnd,
                   C, CC, isTransient, LM, st->downsample, silence, st->arch);

    c = 0;
    do  {
        st->postfilter_period = IMAX(st->postfilter_period, COMBFILTER_MINPERIOD);
        st->postfilter_period_old = IMAX(st->postfilter_period_old, COMBFILTER_MINPERIOD);
        comb_filter(out_syn[c], out_syn[c], st->postfilter_period_old, st->postfilter_period, mode->shortMdctSize,
                    st->postfilter_gain_old, st->postfilter_gain, st->postfilter_tapset_old, st->postfilter_tapset,
                    mode->window, overlap, st->arch);
        if (LM != 0)
            comb_filter(out_syn[c] + mode->shortMdctSize, out_syn[c] + mode->shortMdctSize, st->postfilter_period, postfilter_pitch, N - mode->shortMdctSize,
                        st->postfilter_gain, postfilter_gain, st->postfilter_tapset, postfilter_tapset,
                        mode->window, overlap, st->arch);

    } while (++c < CC);
    st->postfilter_period_old = st->postfilter_period;
    st->postfilter_gain_old = st->postfilter_gain;
    st->postfilter_tapset_old = st->postfilter_tapset;
    st->postfilter_period = postfilter_pitch;
    st->postfilter_gain = postfilter_gain;
    st->postfilter_tapset = postfilter_tapset;
    if (LM != 0) {
        st->postfilter_period_old = st->postfilter_period;
        st->postfilter_gain_old = st->postfilter_gain;
        st->postfilter_tapset_old = st->postfilter_tapset;
    }

    if (C == 1)
        OPUS_COPY(&oldBandE[nbEBands], oldBandE, nbEBands);

    /* In case start or end were to change */
    if (!isTransient) {
        int16_t max_background_increase;
        OPUS_COPY(oldLogE2, oldLogE, 2 * nbEBands);
        OPUS_COPY(oldLogE, oldBandE, 2 * nbEBands);
        /* In normal circumstances, we only allow the noise floor to increase by
           up to 2.4 dB/second, but when we're in DTX, we allow up to 6 dB
           increase for each update.*/
        if (st->loss_count < 10)
            max_background_increase = M * QCONST16(0.001f, DB_SHIFT);
        else
            max_background_increase = QCONST16(1.f, DB_SHIFT);
        for (i = 0; i < 2 * nbEBands; i++)
            backgroundLogE[i] = MIN16(backgroundLogE[i] + max_background_increase, oldBandE[i]);
    }
    else {
        for (i = 0; i < 2 * nbEBands; i++)
            oldLogE[i] = MIN16(oldLogE[i], oldBandE[i]);
    }
    c = 0;
    do {
        for (i = 0; i < start; i++)
        {
            oldBandE[c * nbEBands + i] = 0;
            oldLogE[c * nbEBands + i] = oldLogE2[c * nbEBands + i] = -QCONST16(28.f, DB_SHIFT);
        }
        for (i = end; i < nbEBands; i++)
        {
            oldBandE[c * nbEBands + i] = 0;
            oldLogE[c * nbEBands + i] = oldLogE2[c * nbEBands + i] = -QCONST16(28.f, DB_SHIFT);
        }
    } while (++c < 2);
    st->rng = dec->rng;

    deemphasis(out_syn, pcm, N, CC, st->downsample, mode->preemph, st->preemph_memD, accum);
    st->loss_count = 0;

    if (ec_tell(dec) > 8 * len)
        return OPUS_INTERNAL_ERROR;
    if (ec_get_error(dec))
        st->error = 1;
    return frame_size / st->downsample;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_decoder_ctl(CELTDecoder *__restrict__ st, int request, ...) {
    va_list ap;

    va_start(ap, request);
    switch (request) {
        case CELT_SET_START_BAND_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 0 || value >= st->mode->nbEBands) goto bad_arg;
            st->start = value;
        } break;
        case CELT_SET_END_BAND_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 1 || value > st->mode->nbEBands) goto bad_arg;
            st->end = value;
        } break;
        case CELT_SET_CHANNELS_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 1 || value > 2) goto bad_arg;
            st->stream_channels = value;
        } break;
        case CELT_GET_AND_CLEAR_ERROR_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (value == NULL) goto bad_arg;
            *value = st->error;
            st->error = 0;
        } break;
        case OPUS_GET_LOOKAHEAD_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (value == NULL) goto bad_arg;
            *value = st->overlap / st->downsample;
        } break;
        case OPUS_RESET_STATE: {
            int i;
            int16_t *lpc, *oldBandE, *oldLogE, *oldLogE2;
            lpc = (int16_t *)(st->_decode_mem + (DECODE_BUFFER_SIZE + st->overlap) * st->channels);
            oldBandE = lpc + st->channels * LPC_ORDER;
            oldLogE = oldBandE + 2 * st->mode->nbEBands;
            oldLogE2 = oldLogE + 2 * st->mode->nbEBands;
            OPUS_CLEAR((char *)&st->DECODER_RESET_START, opus_custom_decoder_get_size(st->mode, st->channels) -
                                                             ((char *)&st->DECODER_RESET_START - (char *)st));
            for (i = 0; i < 2 * st->mode->nbEBands; i++) oldLogE[i] = oldLogE2[i] = -QCONST16(28.f, DB_SHIFT);
            st->skip_plc = 1;
        } break;
        case OPUS_GET_PITCH_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (value == NULL) goto bad_arg;
            *value = st->postfilter_period;
        } break;
        case CELT_GET_MODE_REQUEST: {
            const CELTMode **value = va_arg(ap, const CELTMode **);
            if (value == 0) goto bad_arg;
            *value = st->mode;
        } break;
        case CELT_SET_SIGNALLING_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            st->signalling = value;
        } break;
        case OPUS_GET_FINAL_RANGE_REQUEST: {
            uint32_t *value = va_arg(ap, uint32_t *);
            if (value == 0) goto bad_arg;
            *value = st->rng;
        } break;
        case OPUS_SET_PHASE_INVERSION_DISABLED_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 0 || value > 1) {
                goto bad_arg;
            }
            st->disable_inv = value;
        } break;
        case OPUS_GET_PHASE_INVERSION_DISABLED_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            *value = st->disable_inv;
        } break;
        default:
            goto bad_request;
    }
    va_end(ap);
    return OPUS_OK;
bad_arg:
    va_end(ap);
    return OPUS_BAD_ARG;
bad_request:
    va_end(ap);
    return OPUS_UNIMPLEMENTED;
}
//----------------------------------------------------------------------------------------------------------------------

int opus_custom_encoder_get_size(const CELTMode *mode, int channels) {
    int size = sizeof(struct CELTEncoder) + (channels * mode->overlap - 1) * sizeof(int32_t) /* int32_t in_mem[channels*mode->overlap]; */
               + channels * COMBFILTER_MAXPERIOD * sizeof(int32_t)                           /* int32_t prefilter_mem[channels*COMBFILTER_MAXPERIOD]; */
               + 4 * channels * mode->nbEBands * sizeof(int16_t);                            /* int16_t oldBandE[channels*mode->nbEBands]; */
                                                                                             /* int16_t oldLogE[channels*mode->nbEBands]; */
                                                                                             /* int16_t oldLogE2[channels*mode->nbEBands]; */
                                                                                             /* int16_t energyError[channels*mode->nbEBands]; */
    return size;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_encoder_get_size(int channels){
    CELTMode *mode = opus_custom_mode_create(48000, 960, NULL);
    return opus_custom_encoder_get_size(mode, channels);
}
//----------------------------------------------------------------------------------------------------------------------

static int opus_custom_encoder_init_arch(CELTEncoder *st, const CELTMode *mode, int channels, int arch) {
    if (channels < 0 || channels > 2)
        return OPUS_BAD_ARG;

    if (st == NULL || mode == NULL)
        return OPUS_ALLOC_FAIL;

    OPUS_CLEAR((char *)st, opus_custom_encoder_get_size(mode, channels));

    st->mode = mode;
    st->stream_channels = st->channels = channels;

    st->upsample = 1;
    st->start = 0;
    st->end = st->mode->effEBands;
    st->signalling = 1;
    st->arch = arch;

    st->constrained_vbr = 1;
    st->clip = 1;

    st->bitrate = OPUS_BITRATE_MAX;
    st->vbr = 0;
    st->force_intra = 0;
    st->complexity = 5;
    st->lsb_depth = 24;

    celt_encoder_ctl(st, OPUS_RESET_STATE);

    return OPUS_OK;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_encoder_init(CELTEncoder *st, int32_t sampling_rate, int channels,  int arch) {
    int ret;
    ret = opus_custom_encoder_init_arch(st,
                                        opus_custom_mode_create(48000, 960, NULL), channels, arch);
    if (ret != OPUS_OK)
        return ret;
    st->upsample = resampling_factor(sampling_rate);
    return OPUS_OK;
}
//----------------------------------------------------------------------------------------------------------------------

static int transient_analysis(const int32_t *__restrict__ in, int len, int C, int16_t *tf_estimate, int *tf_chan,
                              int allow_weak_transients, int *weak_transient) {
    int i;
    VARDECL(int16_t, tmp);
    int32_t mem0, mem1;
    int is_transient = 0;
    int32_t mask_metric = 0;
    int c;
    int16_t tf_max;
    int len2;
    /* Forward masking: 6.7 dB/ms. */

    int forward_shift = 4;

    /* Table of 6*64/x, trained on real data to minimize the average error */
    static const unsigned char inv_table[128] = {
        255, 255, 156, 110, 86, 70, 59, 51, 45, 40, 37, 33, 31, 28, 26, 25, 23, 22, 21, 20, 19, 18, 17, 16, 16, 15,
        15,  14,  13,  13,  12, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9,  9,  9,  9,  9,  9,  8,  8,  8,  8,  8,  7,
        7,   7,   7,   7,   7,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  5,  5,  5,  5,  5,
        5,   5,   5,   5,   5,  5,  5,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
        4,   4,   4,   4,   4,  4,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  2,
    };
    SAVE_STACK;
    ALLOC(tmp, len, int16_t);

    *weak_transient = 0;
    /* For lower bitrates, let's be more conservative and have a forward masking
       decay of 3.3 dB/ms. This avoids having to code transients at very low
       bitrate (mostly for hybrid), which can result in unstable energy and/or
       partial collapse. */
    if (allow_weak_transients) {
        forward_shift = 5;
    }
    len2 = len / 2;
    for (c = 0; c < C; c++) {
        int32_t mean;
        int32_t unmask = 0;
        int32_t norm;
        int16_t maxE;
        mem0 = 0;
        mem1 = 0;
        /* High-pass filter: (1 - 2*z^-1 + z^-2) / (1 - z^-1 + .5*z^-2) */
        for (i = 0; i < len; i++) {
            int32_t x, y;
            x = SHR32(in[i + c * len], 12);
            y = ADD32(mem0, x);

            mem0 = mem1 + y - SHL32(x, 1);
            mem1 = x - SHR32(y, 1);

            tmp[i] = SROUND16(y, 2);
            /*printf("%f ", tmp[i]);*/
        }
        /*printf("\n");*/
        /* First few samples are bad because we don't propagate the memory */
        OPUS_CLEAR(tmp, 12);

        /* Normalize tmp to max range */
        {
            int shift = 0;
            shift = 14 - celt_ilog2(MAX16(1, celt_maxabs16(tmp, len)));
            if (shift != 0) {
                for (i = 0; i < len; i++) tmp[i] = SHL16(tmp[i], shift);
            }
        }

        mean = 0;
        mem0 = 0;
        /* Grouping by two to reduce complexity */
        /* Forward pass to compute the post-echo threshold*/
        for (i = 0; i < len2; i++) {
            int16_t x2 = PSHR32(MULT16_16(tmp[2 * i], tmp[2 * i]) + MULT16_16(tmp[2 * i + 1], tmp[2 * i + 1]), 16);
            mean += x2;

            /* FIXME: Use PSHR16() instead */
            tmp[i] = mem0 + PSHR32(x2 - mem0, forward_shift);

            mem0 = tmp[i];
        }

        mem0 = 0;
        maxE = 0;
        /* Backward pass to compute the pre-echo threshold */
        for (i = len2 - 1; i >= 0; i--) {
            /* Backward masking: 13.9 dB/ms. */

            /* FIXME: Use PSHR16() instead */
            tmp[i] = mem0 + PSHR32(tmp[i] - mem0, 3);

            mem0 = tmp[i];
            maxE = MAX16(maxE, mem0);
        }
        /*for (i=0;i<len2;i++)printf("%f ", tmp[i]/mean);printf("\n");*/

        /* Compute the ratio of the "frame energy" over the harmonic mean of the energy.
           This essentially corresponds to a bitrate-normalized temporal noise-to-mask
           ratio */

        /* As a compromise with the old transient detector, frame energy is the
           geometric mean of the energy and half the max */

        /* Costs two sqrt() to avoid overflows */
        mean = MULT16_16(celt_sqrt(mean), celt_sqrt(MULT16_16(maxE, len2 >> 1)));

        /* Inverse of the mean energy in Q15+6 */
        norm = SHL32(EXTEND32(len2), 6 + 14) / ADD32(EPSILON, SHR32(mean, 1));
        /* Compute harmonic mean discarding the unreliable boundaries
           The data is smooth, so we only take 1/4th of the samples */
        unmask = 0;
        /* We should never see NaNs here. If we find any, then something really bad happened and we better abort
           before it does any damage later on. If these asserts are disabled (no hardening), then the table
           lookup a few lines below (id = ...) is likely to crash dur to an out-of-bounds read. DO NOT FIX
           that crash on NaN since it could result in a worse issue later on. */
        assert(tmp[0] != 0);
        assert(norm != 0);
        for (i = 12; i < len2 - 5; i += 4) {
            int id;

            id = MAX32(0, MIN32(127, MULT16_32_Q15(tmp[i] + EPSILON, norm))); /* Do not round to nearest */

            unmask += inv_table[id];
        }
        /*printf("%d\n", unmask);*/
        /* Normalize, compensate for the 1/4th of the sample and the factor of 6 in the inverse table */
        unmask = 64 * unmask * 4 / (6 * (len2 - 17));
        if (unmask > mask_metric) {
            *tf_chan = c;
            mask_metric = unmask;
        }
    }
    is_transient = mask_metric > 200;
    /* For low bitrates, define "weak transients" that need to be
       handled differently to avoid partial collapse. */
    if (allow_weak_transients && is_transient && mask_metric < 600) {
        is_transient = 0;
        *weak_transient = 1;
    }
    /* Arbitrary metric for VBR boost */
    tf_max = MAX16(0, celt_sqrt(27 * mask_metric) - 42);
    /* *tf_estimate = 1 + MIN16(1, sqrt(MAX16(0, tf_max-30))/20); */
    *tf_estimate =
        celt_sqrt(MAX32(0, SHL32(MULT16_16(QCONST16(0.0069, 14), MIN16(163, tf_max)), 14) - QCONST32(0.139, 28)));
    /*printf("%d %f\n", tf_max, mask_metric);*/

    /*printf("%d %f %d\n", is_transient, (float)*tf_estimate, tf_max);*/
    return is_transient;
}
//----------------------------------------------------------------------------------------------------------------------

/* Looks for sudden increases of energy to decide whether we need to patch the transient decision */
static int patch_transient_decision(int16_t *newE, int16_t *oldE, int nbEBands, int start, int end, int C) {
    int i, c;
    int32_t mean_diff = 0;
    int16_t spread_old[26];
    /* Apply an aggressive (-6 dB/Bark) spreading function to the old frame to
       avoid false detection caused by irrelevant bands */
    if (C == 1) {
        spread_old[start] = oldE[start];
        for (i = start + 1; i < end; i++) spread_old[i] = MAX16(spread_old[i - 1] - QCONST16(1.0f, DB_SHIFT), oldE[i]);
    } else {
        spread_old[start] = MAX16(oldE[start], oldE[start + nbEBands]);
        for (i = start + 1; i < end; i++)
            spread_old[i] = MAX16(spread_old[i - 1] - QCONST16(1.0f, DB_SHIFT), MAX16(oldE[i], oldE[i + nbEBands]));
    }
    for (i = end - 2; i >= start; i--)
        spread_old[i] = MAX16(spread_old[i], spread_old[i + 1] - QCONST16(1.0f, DB_SHIFT));
    /* Compute mean increase */
    c = 0;
    do {
        for (i = IMAX(2, start); i < end - 1; i++) {
            int16_t x1, x2;
            x1 = MAX16(0, newE[i + c * nbEBands]);
            x2 = MAX16(0, spread_old[i]);
            mean_diff = ADD32(mean_diff, EXTEND32(MAX16(0, SUB16(x1, x2))));
        }
    } while (++c < C);
    mean_diff = DIV32(mean_diff, C * (end - 1 - IMAX(2, start)));
    /*printf("%f %f %d\n", mean_diff, max_diff, count);*/
    return mean_diff > QCONST16(1.f, DB_SHIFT);
}
//----------------------------------------------------------------------------------------------------------------------

/** Apply window and compute the MDCT for all sub-frames and all channels in a frame */
static void compute_mdcts(const CELTMode *mode, int shortBlocks, int32_t *__restrict__ in, int32_t *__restrict__ out,
                          int C, int CC, int LM, int upsample, int arch) {
    const int overlap = mode->overlap;
    int N;
    int B;
    int shift;
    int i, b, c;
    if (shortBlocks) {
        B = shortBlocks;
        N = mode->shortMdctSize;
        shift = mode->maxLM;
    } else {
        B = 1;
        N = mode->shortMdctSize << LM;
        shift = mode->maxLM - LM;
    }
    c = 0;
    do {
        for (b = 0; b < B; b++) {
            /* Interleaving the sub-frames while doing the MDCTs */
            clt_mdct_forward(&mode->mdct, in + c * (B * N + overlap) + b * N, &out[b + c * N * B], mode->window,
                             overlap, shift, B, arch);
        }
    } while (++c < CC);
    if (CC == 2 && C == 1) {
        for (i = 0; i < B * N; i++) out[i] = ADD32(HALF32(out[i]), HALF32(out[B * N + i]));
    }
    if (upsample != 1) {
        c = 0;
        do {
            int bound = B * N / upsample;
            for (i = 0; i < bound; i++) out[c * B * N + i] *= upsample;
            OPUS_CLEAR(&out[c * B * N + bound], B * N - bound);
        } while (++c < C);
    }
}
//----------------------------------------------------------------------------------------------------------------------

void celt_preemphasis(const int16_t *__restrict__ pcmp, int32_t *__restrict__ inp, int N, int CC, int upsample,
                      const int16_t *coef, int32_t *mem, int clip) {
    int i;
    int16_t coef0;
    int32_t m;
    int Nu;

    coef0 = coef[0];
    m = *mem;

    /* Fast path for the normal 48kHz case and no clipping */
    if (coef[1] == 0 && upsample == 1 && !clip) {
        for (i = 0; i < N; i++) {
            int16_t x;
            x = SCALEIN(pcmp[CC * i]);
            /* Apply pre-emphasis */
            inp[i] = SHL32(x, 12) - m;
            m = SHR32(MULT16_16(coef0, x), 15 - 12);
        }
        *mem = m;
        return;
    }

    Nu = N / upsample;
    if (upsample != 1) {
        OPUS_CLEAR(inp, N);
    }
    for (i = 0; i < Nu; i++)
        inp[i * upsample] = SCALEIN(pcmp[CC * i]);

    (void)clip; /* Avoids a warning about clip being unused. */

    {
        for (i = 0; i < N; i++)
        {
            int16_t x;
            x = inp[i];
            /* Apply pre-emphasis */
            inp[i] = SHL32(x, 12) - m;
            m = SHR32(MULT16_16(coef0, x), 15 - 12);
        }
    }
    *mem = m;
}
//----------------------------------------------------------------------------------------------------------------------

static int32_t l1_metric(const int16_t *tmp, int N, int LM, int16_t bias)
{
    int i;
    int32_t L1;
    L1 = 0;
    for (i = 0; i < N; i++)
        L1 += EXTEND32(ABS16(tmp[i]));
    /* When in doubt, prefer good freq resolution */
    L1 = MAC16_32_Q15(L1, LM * bias, L1);
    return L1;
}
//----------------------------------------------------------------------------------------------------------------------

static int tf_analysis(const CELTMode *m, int len, int isTransient, int *tf_res, int lambda, int16_t *X, int N0, int LM,
                       int16_t tf_estimate, int tf_chan, int *importance) {
    int i;
    VARDECL(int, metric);
    int cost0;
    int cost1;
    VARDECL(int, path0);
    VARDECL(int, path1);
    VARDECL(int16_t, tmp);
    VARDECL(int16_t, tmp_1);
    int sel;
    int selcost[2];
    int tf_select = 0;
    int16_t bias;

    SAVE_STACK;
    bias = MULT16_16_Q14(QCONST16(.04f, 15), MAX16(-QCONST16(.25f, 14), QCONST16(.5f, 14) - tf_estimate));
    /*printf("%f ", bias);*/

    ALLOC(metric, len, int);
    ALLOC(tmp, (m->eBands[len] - m->eBands[len - 1]) << LM, int16_t);
    ALLOC(tmp_1, (m->eBands[len] - m->eBands[len - 1]) << LM, int16_t);
    ALLOC(path0, len, int);
    ALLOC(path1, len, int);

    for (i = 0; i < len; i++) {
        int k, N;
        int narrow;
        int32_t L1, best_L1;
        int best_level = 0;
        N = (m->eBands[i + 1] - m->eBands[i]) << LM;
        /* band is too narrow to be split down to LM=-1 */
        narrow = (m->eBands[i + 1] - m->eBands[i]) == 1;
        OPUS_COPY(tmp, &X[tf_chan * N0 + (m->eBands[i] << LM)], N);
        /* Just add the right channel if we're in stereo */
        /*if (C==2)
           for (j=0;j<N;j++)
              tmp[j] = ADD16(SHR16(tmp[j], 1),SHR16(X[N0+j+(m->eBands[i]<<LM)],
           1));*/
        L1 = l1_metric(tmp, N, isTransient ? LM : 0, bias);
        best_L1 = L1;
        /* Check the -1 case for transients */
        if (isTransient && !narrow) {
            OPUS_COPY(tmp_1, tmp, N);
            haar1(tmp_1, N >> LM, 1 << LM);
            L1 = l1_metric(tmp_1, N, LM + 1, bias);
            if (L1 < best_L1) {
                best_L1 = L1;
                best_level = -1;
            }
        }
        /*printf ("%f ", L1);*/
        for (k = 0; k < LM + !(isTransient || narrow); k++) {
            int B;

            if (isTransient)
                B = (LM - k - 1);
            else
                B = k + 1;

            haar1(tmp, N >> k, 1 << k);

            L1 = l1_metric(tmp, N, B, bias);

            if (L1 < best_L1) {
                best_L1 = L1;
                best_level = k + 1;
            }
        }
        /*printf ("%d ", isTransient ? LM-best_level : best_level);*/
        /* metric is in Q1 to be able to select the mid-point (-0.5) for narrower
         * bands */
        if (isTransient)
            metric[i] = 2 * best_level;
        else
            metric[i] = -2 * best_level;
        /* For bands that can't be split to -1, set the metric to the half-way point
           to avoid biasing the decision */
        if (narrow && (metric[i] == 0 || metric[i] == -2 * LM)) metric[i] -= 1;
        /*printf("%d ", metric[i]/2 + (!isTransient)*LM);*/
    }
    /*printf("\n");*/
    /* Search for the optimal tf resolution, including tf_select */
    tf_select = 0;
    for (sel = 0; sel < 2; sel++) {
        cost0 = importance[0] * abs(metric[0] - 2 * tf_select_table[LM][4 * isTransient + 2 * sel + 0]);
        cost1 = importance[0] * abs(metric[0] - 2 * tf_select_table[LM][4 * isTransient + 2 * sel + 1]) +
                (isTransient ? 0 : lambda);
        for (i = 1; i < len; i++) {
            int curr0, curr1;
            curr0 = IMIN(cost0, cost1 + lambda);
            curr1 = IMIN(cost0 + lambda, cost1);
            cost0 = curr0 + importance[i] * abs(metric[i] - 2 * tf_select_table[LM][4 * isTransient + 2 * sel + 0]);
            cost1 = curr1 + importance[i] * abs(metric[i] - 2 * tf_select_table[LM][4 * isTransient + 2 * sel + 1]);
        }
        cost0 = IMIN(cost0, cost1);
        selcost[sel] = cost0;
    }
    /* For now, we're conservative and only allow tf_select=1 for transients.
     * If tests confirm it's useful for non-transients, we could allow it. */
    if (selcost[1] < selcost[0] && isTransient) tf_select = 1;
    cost0 = importance[0] * abs(metric[0] - 2 * tf_select_table[LM][4 * isTransient + 2 * tf_select + 0]);
    cost1 = importance[0] * abs(metric[0] - 2 * tf_select_table[LM][4 * isTransient + 2 * tf_select + 1]) +
            (isTransient ? 0 : lambda);
    /* Viterbi forward pass */
    for (i = 1; i < len; i++) {
        int curr0, curr1;
        int from0, from1;

        from0 = cost0;
        from1 = cost1 + lambda;
        if (from0 < from1) {
            curr0 = from0;
            path0[i] = 0;
        } else {
            curr0 = from1;
            path0[i] = 1;
        }

        from0 = cost0 + lambda;
        from1 = cost1;
        if (from0 < from1) {
            curr1 = from0;
            path1[i] = 0;
        } else {
            curr1 = from1;
            path1[i] = 1;
        }
        cost0 = curr0 + importance[i] * abs(metric[i] - 2 * tf_select_table[LM][4 * isTransient + 2 * tf_select + 0]);
        cost1 = curr1 + importance[i] * abs(metric[i] - 2 * tf_select_table[LM][4 * isTransient + 2 * tf_select + 1]);
    }
    tf_res[len - 1] = cost0 < cost1 ? 0 : 1;
    /* Viterbi backward pass to check the decisions */
    for (i = len - 2; i >= 0; i--) {
        if (tf_res[i + 1] == 1)
            tf_res[i] = path1[i + 1];
        else
            tf_res[i] = path0[i + 1];
    }
    /*printf("%d %f\n", *tf_sum, tf_estimate);*/

    return tf_select;
}
//----------------------------------------------------------------------------------------------------------------------

static void tf_encode(int start, int end, int isTransient, int *tf_res, int LM, int tf_select, ec_enc *enc) {
    int curr, i;
    int tf_select_rsv;
    int tf_changed;
    int logp;
    uint32_t budget;
    uint32_t tell;
    budget = enc->storage * 8;
    tell = ec_tell(enc);
    logp = isTransient ? 2 : 4;
    /* Reserve space to code the tf_select decision. */
    tf_select_rsv = LM > 0 && tell + logp + 1 <= budget;
    budget -= tf_select_rsv;
    curr = tf_changed = 0;
    for (i = start; i < end; i++) {
        if (tell + logp <= budget) {
            ec_enc_bit_logp(enc, tf_res[i] ^ curr, logp);
            tell = ec_tell(enc);
            curr = tf_res[i];
            tf_changed |= curr;
        } else
            tf_res[i] = curr;
        logp = isTransient ? 4 : 5;
    }
    /* Only code tf_select if it would actually make a difference. */
    if (tf_select_rsv &&
        tf_select_table[LM][4 * isTransient + 0 + tf_changed] != tf_select_table[LM][4 * isTransient + 2 + tf_changed])
        ec_enc_bit_logp(enc, tf_select, 1);
    else
        tf_select = 0;
    for (i = start; i < end; i++) tf_res[i] = tf_select_table[LM][4 * isTransient + 2 * tf_select + tf_res[i]];
    /*for(i=0;i<end;i++)printf("%d ", isTransient ? tf_res[i] : LM+tf_res[i]);printf("\n");*/
}
//----------------------------------------------------------------------------------------------------------------------

static int alloc_trim_analysis(const CELTMode *m, const int16_t *X, const int16_t *bandLogE, int end, int LM, int C,
                               int N0, AnalysisInfo *analysis, int16_t *stereo_saving, int16_t tf_estimate,
                               int intensity, int16_t surround_trim, int32_t equiv_rate, int arch) {
    int i;
    int32_t diff = 0;
    int c;
    int trim_index;
    int16_t trim = QCONST16(5.f, 8);
    int16_t logXC, logXC2;
    /* At low bitrate, reducing the trim seems to help. At higher bitrates, it's less
       clear what's best, so we're keeping it as it was before, at least for now. */
    if (equiv_rate < 64000) {
        trim = QCONST16(4.f, 8);
    } else if (equiv_rate < 80000) {
        int32_t frac = (equiv_rate - 64000) >> 10;
        trim = QCONST16(4.f, 8) + QCONST16(1.f / 16.f, 8) * frac;
    }
    if (C == 2) {
        int16_t sum = 0; /* Q10 */
        int16_t minXC;   /* Q10 */
        /* Compute inter-channel correlation for low frequencies */
        for (i = 0; i < 8; i++) {
            int32_t partial;
            partial = celt_inner_prod(&X[m->eBands[i] << LM], &X[N0 + (m->eBands[i] << LM)],
                                      (m->eBands[i + 1] - m->eBands[i]) << LM, arch);
            sum = ADD16(sum, EXTRACT16(SHR32(partial, 18)));
        }
        sum = MULT16_16_Q15(QCONST16(1.f / 8, 15), sum);
        sum = MIN16(QCONST16(1.f, 10), ABS16(sum));
        minXC = sum;
        for (i = 8; i < intensity; i++) {
            int32_t partial;
            partial = celt_inner_prod(&X[m->eBands[i] << LM], &X[N0 + (m->eBands[i] << LM)],
                                      (m->eBands[i + 1] - m->eBands[i]) << LM, arch);
            minXC = MIN16(minXC, ABS16(EXTRACT16(SHR32(partial, 18))));
        }
        minXC = MIN16(QCONST16(1.f, 10), ABS16(minXC));
        /*printf ("%f\n", sum);*/
        /* mid-side savings estimations based on the LF average*/
        logXC = celt_log2(QCONST32(1.001f, 20) - MULT16_16(sum, sum));
        /* mid-side savings estimations based on min correlation */
        logXC2 = MAX16(HALF16(logXC), celt_log2(QCONST32(1.001f, 20) - MULT16_16(minXC, minXC)));

        /* Compensate for Q20 vs Q14 input and convert output to Q8 */
        logXC = PSHR32(logXC - QCONST16(6.f, DB_SHIFT), DB_SHIFT - 8);
        logXC2 = PSHR32(logXC2 - QCONST16(6.f, DB_SHIFT), DB_SHIFT - 8);

        trim += MAX16(-QCONST16(4.f, 8), MULT16_16_Q15(QCONST16(.75f, 15), logXC));
        *stereo_saving = MIN16(*stereo_saving + QCONST16(0.25f, 8), -HALF16(logXC2));
    }

    /* Estimate spectral tilt */
    c = 0;
    do {
        for (i = 0; i < end - 1; i++) {
            diff += bandLogE[i + c * m->nbEBands] * (int32_t)(2 + 2 * i - end);
        }
    } while (++c < C);
    diff /= C * (end - 1);
    /*printf("%f\n", diff);*/
    trim -= MAX32(-QCONST16(2.f, 8), MIN32(QCONST16(2.f, 8), SHR32(diff + QCONST16(1.f, DB_SHIFT), DB_SHIFT - 8) / 6));
    trim -= SHR16(surround_trim, DB_SHIFT - 8);
    trim -= 2 * SHR16(tf_estimate, 14 - 8);

    (void)analysis;

    trim_index = PSHR32(trim, 8);

    trim_index = IMAX(0, IMIN(10, trim_index));
    /*printf("%d\n", trim_index);*/

    return trim_index;
}
//----------------------------------------------------------------------------------------------------------------------

static int stereo_analysis(const CELTMode *m, const int16_t *X, int LM, int N0) {
    int i;
    int thetas;
    int32_t sumLR = EPSILON, sumMS = EPSILON;

    /* Use the L1 norm to model the entropy of the L/R signal vs the M/S signal */
    for (i = 0; i < 13; i++) {
        int j;
        for (j = m->eBands[i] << LM; j < m->eBands[i + 1] << LM; j++) {
            int32_t L, R, M, S;
            /* We cast to 32-bit first because of the -32768 case */
            L = EXTEND32(X[j]);
            R = EXTEND32(X[N0 + j]);
            M = ADD32(L, R);
            S = SUB32(L, R);
            sumLR = ADD32(sumLR, ADD32(ABS32(L), ABS32(R)));
            sumMS = ADD32(sumMS, ADD32(ABS32(M), ABS32(S)));
        }
    }
    sumMS = MULT16_32_Q15(QCONST16(0.707107f, 15), sumMS);
    thetas = 13;
    /* We don't need thetas for lower bands with LM<=1 */
    if (LM <= 1) thetas -= 8;
    return MULT16_32_Q15((m->eBands[13] << (LM + 1)) + thetas, sumMS) > MULT16_32_Q15(m->eBands[13] << (LM + 1), sumLR);
}
//----------------------------------------------------------------------------------------------------------------------

#define MSWAP(a, b)      \
    do {                 \
        int16_t tmp = a; \
        a = b;           \
        b = tmp;         \
    } while (0)

static int16_t median_of_5(const int16_t *x) {
    int16_t t0, t1, t2, t3, t4;
    t2 = x[2];
    if (x[0] > x[1]) {
        t0 = x[1];
        t1 = x[0];
    } else {
        t0 = x[0];
        t1 = x[1];
    }
    if (x[3] > x[4]) {
        t3 = x[4];
        t4 = x[3];
    } else {
        t3 = x[3];
        t4 = x[4];
    }
    if (t0 > t3) {
        MSWAP(t0, t3);
        MSWAP(t1, t4);
    }
    if (t2 > t1) {
        if (t1 < t3)
            return MIN16(t2, t3);
        else
            return MIN16(t4, t1);
    } else {
        if (t2 < t3)
            return MIN16(t1, t3);
        else
            return MIN16(t2, t4);
    }
}
//----------------------------------------------------------------------------------------------------------------------

static int16_t median_of_3(const int16_t *x) {
    int16_t t0, t1, t2;
    if (x[0] > x[1]) {
        t0 = x[1];
        t1 = x[0];
    } else {
        t0 = x[0];
        t1 = x[1];
    }
    t2 = x[2];
    if (t1 < t2)
        return t1;
    else if (t0 < t2)
        return t2;
    else
        return t0;
}
//----------------------------------------------------------------------------------------------------------------------

static int16_t dynalloc_analysis(const int16_t *bandLogE, const int16_t *bandLogE2, int nbEBands, int start, int end,
                                 int C, int *offsets, int lsb_depth, const int16_t *logN, int isTransient, int vbr,
                                 int constrained_vbr, const int16_t *eBands, int LM, int effectiveBytes,
                                 int32_t *tot_boost_, int lfe, int16_t *surround_dynalloc, AnalysisInfo *analysis,
                                 int *importance, int *spread_weight) {
    int i, c;
    int32_t tot_boost = 0;
    int16_t maxDepth;
    VARDECL(int16_t, follower);
    VARDECL(int16_t, noise_floor);
    SAVE_STACK;
    ALLOC(follower, C * nbEBands, int16_t);
    ALLOC(noise_floor, C * nbEBands, int16_t);
    OPUS_CLEAR(offsets, nbEBands);
    /* Dynamic allocation code */
    maxDepth = -QCONST16(31.9f, DB_SHIFT);
    for (i = 0; i < end; i++) {
        /* Noise floor must take into account eMeans, the depth, the width of the bands
           and the preemphasis filter (approx. square of bark band ID) */
        noise_floor[i] = MULT16_16(QCONST16(0.0625f, DB_SHIFT), logN[i]) + QCONST16(.5f, DB_SHIFT) +
                         SHL16(9 - lsb_depth, DB_SHIFT) - SHL16(eMeans[i], 6) +
                         MULT16_16(QCONST16(.0062, DB_SHIFT), (i + 5) * (i + 5));
    }
    c = 0;
    do {
        for (i = 0; i < end; i++) maxDepth = MAX16(maxDepth, bandLogE[c * nbEBands + i] - noise_floor[i]);
    } while (++c < C);
    {
        /* Compute a really simple masking model to avoid taking into account completely masked
           bands when computing the spreading decision. */
        VARDECL(int16_t, mask);
        VARDECL(int16_t, sig);
        ALLOC(mask, nbEBands, int16_t);
        ALLOC(sig, nbEBands, int16_t);
        for (i = 0; i < end; i++) mask[i] = bandLogE[i] - noise_floor[i];
        if (C == 2) {
            for (i = 0; i < end; i++) mask[i] = MAX16(mask[i], bandLogE[nbEBands + i] - noise_floor[i]);
        }
        OPUS_COPY(sig, mask, end);
        for (i = 1; i < end; i++) mask[i] = MAX16(mask[i], mask[i - 1] - QCONST16(2.f, DB_SHIFT));
        for (i = end - 2; i >= 0; i--) mask[i] = MAX16(mask[i], mask[i + 1] - QCONST16(3.f, DB_SHIFT));
        for (i = 0; i < end; i++) {
            /* Compute SMR: Mask is never more than 72 dB below the peak and never below the noise floor.*/
            int16_t smr = sig[i] - MAX16(MAX16(0, maxDepth - QCONST16(12.f, DB_SHIFT)), mask[i]);
            /* Clamp SMR to make sure we're not shifting by something negative or too large. */

            /* FIXME: Use PSHR16() instead */
            int shift = -PSHR32(MAX16(-QCONST16(5.f, DB_SHIFT), MIN16(0, smr)), DB_SHIFT);

            spread_weight[i] = 32 >> shift;
        }
        /*for (i=0;i<end;i++)
           printf("%d ", spread_weight[i]);
        printf("\n");*/
    }
    /* Make sure that dynamic allocation can't make us bust the budget */
    if (effectiveBytes > 50 && LM >= 1 && !lfe) {
        int last = 0;
        c = 0;
        do {
            int16_t offset;
            int16_t tmp;
            int16_t *f;
            f = &follower[c * nbEBands];
            f[0] = bandLogE2[c * nbEBands];
            for (i = 1; i < end; i++) {
                /* The last band to be at least 3 dB higher than the previous one
                   is the last we'll consider. Otherwise, we run into problems on
                   bandlimited signals. */
                if (bandLogE2[c * nbEBands + i] > bandLogE2[c * nbEBands + i - 1] + QCONST16(.5f, DB_SHIFT)) last = i;
                f[i] = MIN16(f[i - 1] + QCONST16(1.5f, DB_SHIFT), bandLogE2[c * nbEBands + i]);
            }
            for (i = last - 1; i >= 0; i--)
                f[i] = MIN16(f[i], MIN16(f[i + 1] + QCONST16(2.f, DB_SHIFT), bandLogE2[c * nbEBands + i]));

            /* Combine with a median filter to avoid dynalloc triggering unnecessarily.
               The "offset" value controls how conservative we are -- a higher offset
               reduces the impact of the median filter and makes dynalloc use more bits. */
            offset = QCONST16(1.f, DB_SHIFT);
            for (i = 2; i < end - 2; i++) f[i] = MAX16(f[i], median_of_5(&bandLogE2[c * nbEBands + i - 2]) - offset);
            tmp = median_of_3(&bandLogE2[c * nbEBands]) - offset;
            f[0] = MAX16(f[0], tmp);
            f[1] = MAX16(f[1], tmp);
            tmp = median_of_3(&bandLogE2[c * nbEBands + end - 3]) - offset;
            f[end - 2] = MAX16(f[end - 2], tmp);
            f[end - 1] = MAX16(f[end - 1], tmp);

            for (i = 0; i < end; i++) f[i] = MAX16(f[i], noise_floor[i]);
        } while (++c < C);
        if (C == 2) {
            for (i = start; i < end; i++) {
                /* Consider 24 dB "cross-talk" */
                follower[nbEBands + i] = MAX16(follower[nbEBands + i], follower[i] - QCONST16(4.f, DB_SHIFT));
                follower[i] = MAX16(follower[i], follower[nbEBands + i] - QCONST16(4.f, DB_SHIFT));
                follower[i] = HALF16(MAX16(0, bandLogE[i] - follower[i]) +
                                     MAX16(0, bandLogE[nbEBands + i] - follower[nbEBands + i]));
            }
        } else {
            for (i = start; i < end; i++) {
                follower[i] = MAX16(0, bandLogE[i] - follower[i]);
            }
        }
        for (i = start; i < end; i++) follower[i] = MAX16(follower[i], surround_dynalloc[i]);
        for (i = start; i < end; i++) {
            importance[i] = PSHR32(13 * celt_exp2(MIN16(follower[i], QCONST16(4.f, DB_SHIFT))), 16);
        }
        /* For non-transient CBR/CVBR frames, halve the dynalloc contribution */
        if ((!vbr || constrained_vbr) && !isTransient) {
            for (i = start; i < end; i++) follower[i] = HALF16(follower[i]);
        }
        for (i = start; i < end; i++) {
            if (i < 8) follower[i] *= 2;
            if (i >= 12) follower[i] = HALF16(follower[i]);
        }

        (void)analysis;

        for (i = start; i < end; i++) {
            int width;
            int boost;
            int boost_bits;

            follower[i] = MIN16(follower[i], QCONST16(4, DB_SHIFT));

            width = C * (eBands[i + 1] - eBands[i]) << LM;
            if (width < 6) {
                boost = (int)SHR32(EXTEND32(follower[i]), DB_SHIFT);
                boost_bits = boost * width << BITRES;
            } else if (width > 48) {
                boost = (int)SHR32(EXTEND32(follower[i]) * 8, DB_SHIFT);
                boost_bits = (boost * width << BITRES) / 8;
            } else {
                boost = (int)SHR32(EXTEND32(follower[i]) * width / 6, DB_SHIFT);
                boost_bits = boost * 6 << BITRES;
            }
            /* For CBR and non-transient CVBR frames, limit dynalloc to 2/3 of the bits */
            if ((!vbr || (constrained_vbr && !isTransient)) &&
                (tot_boost + boost_bits) >> BITRES >> 3 > 2 * effectiveBytes / 3) {
                int32_t cap = ((2 * effectiveBytes / 3) << BITRES << 3);
                offsets[i] = cap - tot_boost;
                tot_boost = cap;
                break;
            } else {
                offsets[i] = boost;
                tot_boost += boost_bits;
            }
        }
    } else {
        for (i = start; i < end; i++) importance[i] = 13;
    }
    *tot_boost_ = tot_boost;

    return maxDepth;
}
//----------------------------------------------------------------------------------------------------------------------

static int run_prefilter(CELTEncoder *st, int32_t *in, int32_t *prefilter_mem, int CC, int N, int prefilter_tapset,
                         int *pitch, int16_t *gain, int *qgain, int enabled, int nbAvailableBytes,
                         AnalysisInfo *analysis) {
    int c;
    VARDECL(int32_t, _pre);
    int32_t *pre[2];
    const CELTMode *mode;
    int pitch_index;
    int16_t gain1;
    int16_t pf_threshold;
    int pf_on;
    int qg;
    int overlap;
    SAVE_STACK;

    mode = st->mode;
    overlap = mode->overlap;
    ALLOC(_pre, CC * (N + COMBFILTER_MAXPERIOD), int32_t);

    pre[0] = _pre;
    pre[1] = _pre + (N + COMBFILTER_MAXPERIOD);

    c = 0;
    do {
        OPUS_COPY(pre[c], prefilter_mem + c * COMBFILTER_MAXPERIOD, COMBFILTER_MAXPERIOD);
        OPUS_COPY(pre[c] + COMBFILTER_MAXPERIOD, in + c * (N + overlap) + overlap, N);
    } while (++c < CC);

    if (enabled) {
        VARDECL(int16_t, pitch_buf);
        ALLOC(pitch_buf, (COMBFILTER_MAXPERIOD + N) >> 1, int16_t);

        pitch_downsample(pre, pitch_buf, COMBFILTER_MAXPERIOD + N, CC, st->arch);
        /* Don't search for the fir last 1.5 octave of the range because
           there's too many false-positives due to short-term correlation */
        pitch_search(pitch_buf + (COMBFILTER_MAXPERIOD >> 1), pitch_buf, N,
                     COMBFILTER_MAXPERIOD - 3 * COMBFILTER_MINPERIOD, &pitch_index, st->arch);
        pitch_index = COMBFILTER_MAXPERIOD - pitch_index;

        gain1 = remove_doubling(pitch_buf, COMBFILTER_MAXPERIOD, COMBFILTER_MINPERIOD, N, &pitch_index,
                                st->prefilter_period, st->prefilter_gain, st->arch);
        if (pitch_index > COMBFILTER_MAXPERIOD - 2) pitch_index = COMBFILTER_MAXPERIOD - 2;
        gain1 = MULT16_16_Q15(QCONST16(.7f, 15), gain1);
        /*printf("%d %d %f %f\n", pitch_change, pitch_index, gain1, st->analysis.tonality);*/
        if (st->loss_rate > 2) gain1 = HALF32(gain1);
        if (st->loss_rate > 4) gain1 = HALF32(gain1);
        if (st->loss_rate > 8) gain1 = 0;
    } else {
        gain1 = 0;
        pitch_index = COMBFILTER_MINPERIOD;
    }

    (void)analysis;

    /* Gain threshold for enabling the prefilter/postfilter */
    pf_threshold = QCONST16(.2f, 15);

    /* Adjusting the threshold based on rate and continuity */
    if (abs(pitch_index - st->prefilter_period) * 10 > pitch_index) pf_threshold += QCONST16(.2f, 15);
    if (nbAvailableBytes < 25) pf_threshold += QCONST16(.1f, 15);
    if (nbAvailableBytes < 35) pf_threshold += QCONST16(.1f, 15);
    if (st->prefilter_gain > QCONST16(.4f, 15)) pf_threshold -= QCONST16(.1f, 15);
    if (st->prefilter_gain > QCONST16(.55f, 15)) pf_threshold -= QCONST16(.1f, 15);

    /* Hard threshold at 0.2 */
    pf_threshold = MAX16(pf_threshold, QCONST16(.2f, 15));
    if (gain1 < pf_threshold) {
        gain1 = 0;
        pf_on = 0;
        qg = 0;
    } else {
        /*This block is not gated by a total bits check only because
          of the nbAvailableBytes check above.*/
        if (ABS16(gain1 - st->prefilter_gain) < QCONST16(.1f, 15)) gain1 = st->prefilter_gain;

        qg = ((gain1 + 1536) >> 10) / 3 - 1;

        qg = IMAX(0, IMIN(7, qg));
        gain1 = QCONST16(0.09375f, 15) * (qg + 1);
        pf_on = 1;
    }
    /*printf("%d %f\n", pitch_index, gain1);*/

    c = 0;
    do {
        int offset = mode->shortMdctSize - overlap;
        st->prefilter_period = IMAX(st->prefilter_period, COMBFILTER_MINPERIOD);
        OPUS_COPY(in + c * (N + overlap), st->in_mem + c * (overlap), overlap);
        if (offset)
            comb_filter(in + c * (N + overlap) + overlap, pre[c] + COMBFILTER_MAXPERIOD, st->prefilter_period,
                        st->prefilter_period, offset, -st->prefilter_gain, -st->prefilter_gain, st->prefilter_tapset,
                        st->prefilter_tapset, NULL, 0, st->arch);

        comb_filter(in + c * (N + overlap) + overlap + offset, pre[c] + COMBFILTER_MAXPERIOD + offset,
                    st->prefilter_period, pitch_index, N - offset, -st->prefilter_gain, -gain1, st->prefilter_tapset,
                    prefilter_tapset, mode->window, overlap, st->arch);
        OPUS_COPY(st->in_mem + c * (overlap), in + c * (N + overlap) + N, overlap);

        if (N > COMBFILTER_MAXPERIOD) {
            OPUS_COPY(prefilter_mem + c * COMBFILTER_MAXPERIOD, pre[c] + N, COMBFILTER_MAXPERIOD);
        } else {
            OPUS_MOVE(prefilter_mem + c * COMBFILTER_MAXPERIOD, prefilter_mem + c * COMBFILTER_MAXPERIOD + N,
                      COMBFILTER_MAXPERIOD - N);
            OPUS_COPY(prefilter_mem + c * COMBFILTER_MAXPERIOD + COMBFILTER_MAXPERIOD - N,
                      pre[c] + COMBFILTER_MAXPERIOD, N);
        }
    } while (++c < CC);

    *gain = gain1;
    *pitch = pitch_index;
    *qgain = qg;
    return pf_on;
}
//----------------------------------------------------------------------------------------------------------------------

static int compute_vbr(const CELTMode *mode, AnalysisInfo *analysis, int32_t base_target, int LM, int32_t bitrate,
                       int lastCodedBands, int C, int intensity, int constrained_vbr, int16_t stereo_saving,
                       int tot_boost, int16_t tf_estimate, int pitch_change, int16_t maxDepth, int lfe,
                       int has_surround_mask, int16_t surround_masking, int16_t temporal_vbr) {
    /* The target rate in 8th bits per frame */
    int32_t target;
    int coded_bins;
    int coded_bands;
    int16_t tf_calibration;
    int nbEBands;
    const int16_t *eBands;

    nbEBands = mode->nbEBands;
    eBands = mode->eBands;

    coded_bands = lastCodedBands ? lastCodedBands : nbEBands;
    coded_bins = eBands[coded_bands] << LM;
    if (C == 2) coded_bins += eBands[IMIN(intensity, coded_bands)] << LM;

    target = base_target;

    /*printf("%f %f %f %f %d %d ", st->analysis.activity, st->analysis.tonality, tf_estimate, st->stereo_saving,
     * tot_boost, coded_bands);*/

    /* Stereo savings */
    if (C == 2) {
        int coded_stereo_bands;
        int coded_stereo_dof;
        int16_t max_frac;
        coded_stereo_bands = IMIN(intensity, coded_bands);
        coded_stereo_dof = (eBands[coded_stereo_bands] << LM) - coded_stereo_bands;
        /* Maximum fraction of the bits we can save if the signal is mono. */
        max_frac = DIV32_16(MULT16_16(QCONST16(0.8f, 15), coded_stereo_dof), coded_bins);
        stereo_saving = MIN16(stereo_saving, QCONST16(1.f, 8));
        /*printf("%d %d %d ", coded_stereo_dof, coded_bins, tot_boost);*/
        target -= (int32_t)MIN32(MULT16_32_Q15(max_frac, target),
                                 SHR32(MULT16_16(stereo_saving - QCONST16(0.1f, 8), (coded_stereo_dof << BITRES)), 8));
    }
    /* Boost the rate according to dynalloc (minus the dynalloc average for calibration). */
    target += tot_boost - (19 << LM);
    /* Apply transient boost, compensating for average boost. */
    tf_calibration = QCONST16(0.044f, 14);
    target += (int32_t)SHL32(MULT16_32_Q15(tf_estimate - tf_calibration, target), 1);

    (void)analysis;
    (void)pitch_change;

    if (has_surround_mask && !lfe) {
        int32_t surround_target = target + (int32_t)SHR32(MULT16_16(surround_masking, coded_bins << BITRES), DB_SHIFT);
        /*printf("%f %d %d %d %d %d %d ", surround_masking, coded_bins, st->end, st->intensity, surround_target, target,
         * st->bitrate);*/
        target = IMAX(target / 4, surround_target);
    }

    {
        int32_t floor_depth;
        int bins;
        bins = eBands[nbEBands - 2] << LM;
        /*floor_depth = SHR32(MULT16_16((C*bins<<BITRES),celt_log2(SHL32(MAX16(1,sample_max),13))), DB_SHIFT);*/
        floor_depth = (int32_t)SHR32(MULT16_16((C * bins << BITRES), maxDepth), DB_SHIFT);
        floor_depth = IMAX(floor_depth, target >> 2);
        target = IMIN(target, floor_depth);
        /*printf("%f %d\n", maxDepth, floor_depth);*/
    }

    /* Make VBR less aggressive for constrained VBR because we can't keep a higher bitrate
       for long. Needs tuning. */
    if ((!has_surround_mask || lfe) && constrained_vbr) {
        target = base_target + (int32_t)MULT16_32_Q15(QCONST16(0.67f, 15), target - base_target);
    }

    if (!has_surround_mask && tf_estimate < QCONST16(.2f, 14)) {
        int16_t amount;
        int16_t tvbr_factor;
        amount = MULT16_16_Q15(QCONST16(.0000031f, 30), IMAX(0, IMIN(32000, 96000 - bitrate)));
        tvbr_factor = SHR32(MULT16_16(temporal_vbr, amount), DB_SHIFT);
        target += (int32_t)MULT16_32_Q15(tvbr_factor, target);
    }

    /* Don't allow more than doubling the rate */
    target = IMIN(2 * base_target, target);

    return target;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_encode_with_ec(CELTEncoder *__restrict__ st, const int16_t *pcm, int frame_size, unsigned char *compressed,
                        int nbCompressedBytes, ec_enc *enc) {
    int i, c, N;
    int32_t bits;
    ec_enc _enc;
    VARDECL(int32_t, in);
    VARDECL(int32_t, freq);
    VARDECL(int16_t, X);
    VARDECL(int32_t, bandE);
    VARDECL(int16_t, bandLogE);
    VARDECL(int16_t, bandLogE2);
    VARDECL(int, fine_quant);
    VARDECL(int16_t, error);
    VARDECL(int, pulses);
    VARDECL(int, cap);
    VARDECL(int, offsets);
    VARDECL(int, importance);
    VARDECL(int, spread_weight);
    VARDECL(int, fine_priority);
    VARDECL(int, tf_res);
    VARDECL(unsigned char, collapse_masks);
    int32_t *prefilter_mem;
    int16_t *oldBandE, *oldLogE, *oldLogE2, *energyError;
    int shortBlocks = 0;
    int isTransient = 0;
    const int CC = st->channels;
    const int C = st->stream_channels;
    int LM, M;
    int tf_select;
    int nbFilledBytes, nbAvailableBytes;
    int start;
    int end;
    int effEnd;
    int codedBands;
    int alloc_trim;
    int pitch_index = COMBFILTER_MINPERIOD;
    int16_t gain1 = 0;
    int dual_stereo = 0;
    int effectiveBytes;
    int dynalloc_logp;
    int32_t vbr_rate;
    int32_t total_bits;
    int32_t total_boost;
    int32_t balance;
    int32_t tell;
    int32_t tell0_frac;
    int prefilter_tapset = 0;
    int pf_on;
    int anti_collapse_rsv;
    int anti_collapse_on = 0;
    int silence = 0;
    int tf_chan = 0;
    int16_t tf_estimate;
    int pitch_change = 0;
    int32_t tot_boost;
    int32_t sample_max;
    int16_t maxDepth;
    const CELTMode *mode;
    int nbEBands;
    int overlap;
    const int16_t *eBands;
    int secondMdct;
    int signalBandwidth;
    int transient_got_disabled = 0;
    int16_t surround_masking = 0;
    int16_t temporal_vbr = 0;
    int16_t surround_trim = 0;
    int32_t equiv_rate;
    int hybrid;
    int weak_transient = 0;
    int enable_tf_analysis;
    VARDECL(int16_t, surround_dynalloc);
    ALLOC_STACK;

    mode = st->mode;
    nbEBands = mode->nbEBands;
    overlap = mode->overlap;
    eBands = mode->eBands;
    start = st->start;
    end = st->end;
    hybrid = start != 0;
    tf_estimate = 0;
    if (nbCompressedBytes < 2 || pcm == NULL) {
        return OPUS_BAD_ARG;
    }

    frame_size *= st->upsample;
    for (LM = 0; LM <= mode->maxLM; LM++)
        if (mode->shortMdctSize << LM == frame_size) break;
    if (LM > mode->maxLM) {
        return OPUS_BAD_ARG;
    }
    M = 1 << LM;
    N = M * mode->shortMdctSize;

    prefilter_mem = st->in_mem + CC * (overlap);
    oldBandE = (int16_t *)(st->in_mem + CC * (overlap + COMBFILTER_MAXPERIOD));
    oldLogE = oldBandE + CC * nbEBands;
    oldLogE2 = oldLogE + CC * nbEBands;
    energyError = oldLogE2 + CC * nbEBands;

    if (enc == NULL) {
        tell0_frac = tell = 1;
        nbFilledBytes = 0;
    } else {
        tell0_frac = ec_tell_frac(enc);
        tell = ec_tell(enc);
        nbFilledBytes = (tell + 4) >> 3;
    }

    celt_assert(st->signalling == 0);

    /* Can't produce more than 1275 output bytes */
    nbCompressedBytes = IMIN(nbCompressedBytes, 1275);
    nbAvailableBytes = nbCompressedBytes - nbFilledBytes;

    if (st->vbr && st->bitrate != OPUS_BITRATE_MAX) {
        int32_t den = mode->Fs >> BITRES;
        vbr_rate = (st->bitrate * frame_size + (den >> 1)) / den;

        effectiveBytes = vbr_rate >> (3 + BITRES);
    } else {
        int32_t tmp;
        vbr_rate = 0;
        tmp = st->bitrate * frame_size;
        if (tell > 1) tmp += tell;
        if (st->bitrate != OPUS_BITRATE_MAX)
            nbCompressedBytes =
                IMAX(2, IMIN(nbCompressedBytes, (tmp + 4 * mode->Fs) / (8 * mode->Fs) - !!st->signalling));
        effectiveBytes = nbCompressedBytes - nbFilledBytes;
    }
    equiv_rate = ((int32_t)nbCompressedBytes * 8 * 50 >> (3 - LM)) - (40 * C + 20) * ((400 >> LM) - 50);
    if (st->bitrate != OPUS_BITRATE_MAX)
        equiv_rate = IMIN(equiv_rate, st->bitrate - (40 * C + 20) * ((400 >> LM) - 50));

    if (enc == NULL) {
        ec_enc_init(&_enc, compressed, nbCompressedBytes);
        enc = &_enc;
    }

    if (vbr_rate > 0) {
        /* Computes the max bit-rate allowed in VBR mode to avoid violating the
            target rate and buffering.
           We must do this up front so that bust-prevention logic triggers
            correctly if we don't have enough bits. */
        if (st->constrained_vbr) {
            int32_t vbr_bound;
            int32_t max_allowed;
            /* We could use any multiple of vbr_rate as bound (depending on the
                delay).
               This is clamped to ensure we use at least two bytes if the encoder
                was entirely empty, but to allow 0 in hybrid mode. */
            vbr_bound = vbr_rate;
            max_allowed = IMIN(IMAX(tell == 1 ? 2 : 0, (vbr_rate + vbr_bound - st->vbr_reservoir) >> (BITRES + 3)),
                               nbAvailableBytes);
            if (max_allowed < nbAvailableBytes) {
                nbCompressedBytes = nbFilledBytes + max_allowed;
                nbAvailableBytes = max_allowed;
                ec_enc_shrink(enc, nbCompressedBytes);
            }
        }
    }
    total_bits = nbCompressedBytes * 8;

    effEnd = end;
    if (effEnd > mode->effEBands) effEnd = mode->effEBands;

    ALLOC(in, CC * (N + overlap), int32_t);

    sample_max = MAX32(st->overlap_max, celt_maxabs16(pcm, C * (N - overlap) / st->upsample));
    st->overlap_max = celt_maxabs16(pcm + C * (N - overlap) / st->upsample, C * overlap / st->upsample);
    sample_max = MAX32(sample_max, st->overlap_max);

    silence = (sample_max == 0);

    if (tell == 1)
        ec_enc_bit_logp(enc, silence, 15);
    else
        silence = 0;
    if (silence) {
        /*In VBR mode there is no need to send more than the minimum. */
        if (vbr_rate > 0) {
            effectiveBytes = nbCompressedBytes = IMIN(nbCompressedBytes, nbFilledBytes + 2);
            total_bits = nbCompressedBytes * 8;
            nbAvailableBytes = 2;
            ec_enc_shrink(enc, nbCompressedBytes);
        }
        /* Pretend we've filled all the remaining bits with zeros
              (that's what the initialiser did anyway) */
        tell = nbCompressedBytes * 8;
        enc->nbits_total += tell - ec_tell(enc);
    }
    c = 0;
    do {
        int need_clip = 0;

        celt_preemphasis(pcm + c, in + c * (N + overlap) + overlap, N, CC, st->upsample, mode->preemph,
                         st->preemph_memE + c, need_clip);
    } while (++c < CC);

    /* Find pitch period and gain */
    {
        int enabled;
        int qg;
        enabled = ((st->lfe && nbAvailableBytes > 3) || nbAvailableBytes > 12 * C) && !hybrid && !silence &&
                  !st->disable_pf && st->complexity >= 5;

        prefilter_tapset = st->tapset_decision;
        pf_on = run_prefilter(st, in, prefilter_mem, CC, N, prefilter_tapset, &pitch_index, &gain1, &qg, enabled,
                              nbAvailableBytes, &st->analysis);
        if ((gain1 > QCONST16(.4f, 15) || st->prefilter_gain > QCONST16(.4f, 15)) &&
            (!st->analysis.valid || st->analysis.tonality > .3L) &&
            (pitch_index > 1.26 * st->prefilter_period || pitch_index < .79 * st->prefilter_period))
            pitch_change = 1;
        if (pf_on == 0) {
            if (!hybrid && tell + 16 <= total_bits) ec_enc_bit_logp(enc, 0, 1);
        } else {
            /*This block is not gated by a total bits check only because
              of the nbAvailableBytes check above.*/
            int octave;
            ec_enc_bit_logp(enc, 1, 1);
            pitch_index += 1;
            octave = EC_ILOG(pitch_index) - 5;
            ec_enc_uint(enc, octave, 6);
            ec_enc_bits(enc, pitch_index - (16 << octave), 4 + octave);
            pitch_index -= 1;
            ec_enc_bits(enc, qg, 3);
            ec_enc_icdf(enc, prefilter_tapset, tapset_icdf, 2);
        }
    }

    isTransient = 0;
    shortBlocks = 0;
    if (st->complexity >= 1 && !st->lfe) {
        /* Reduces the likelihood of energy instability on fricatives at low bitrate
           in hybrid mode. It seems like we still want to have real transients on vowels
           though (small SILK quantization offset value). */
        int allow_weak_transients = hybrid && effectiveBytes < 15 && st->silk_info.signalType != 2;
        isTransient =
            transient_analysis(in, N + overlap, CC, &tf_estimate, &tf_chan, allow_weak_transients, &weak_transient);
    }
    if (LM > 0 && ec_tell(enc) + 3 <= total_bits) {
        if (isTransient) shortBlocks = M;
    } else {
        isTransient = 0;
        transient_got_disabled = 1;
    }

    ALLOC(freq, CC * N, int32_t); /**< Interleaved signal MDCTs */
    ALLOC(bandE, nbEBands * CC, int32_t);
    ALLOC(bandLogE, nbEBands * CC, int16_t);

    secondMdct = shortBlocks && st->complexity >= 8;
    ALLOC(bandLogE2, C * nbEBands, int16_t);
    if (secondMdct) {
        compute_mdcts(mode, 0, in, freq, C, CC, LM, st->upsample, st->arch);
        compute_band_energies(mode, freq, bandE, effEnd, C, LM, st->arch);
        amp2Log2(mode, effEnd, end, bandE, bandLogE2, C);
        for (i = 0; i < C * nbEBands; i++) bandLogE2[i] += HALF16(SHL16(LM, DB_SHIFT));
    }

    compute_mdcts(mode, shortBlocks, in, freq, C, CC, LM, st->upsample, st->arch);
    /* This should catch any NaN in the CELT input. Since we're not supposed to see any (they're filtered
       at the Opus layer), just abort. */
    assert((freq[0] != 0) && (C == 1 || (freq[N] != 0)));
    if (CC == 2 && C == 1) tf_chan = 0;
    compute_band_energies(mode, freq, bandE, effEnd, C, LM, st->arch);

    if (st->lfe) {
        for (i = 2; i < end; i++) {
            bandE[i] = IMIN(bandE[i], MULT16_32_Q15(QCONST16(1e-4f, 15), bandE[0]));
            bandE[i] = MAX32(bandE[i], EPSILON);
        }
    }
    amp2Log2(mode, effEnd, end, bandE, bandLogE, C);

    ALLOC(surround_dynalloc, C * nbEBands, int16_t);
    OPUS_CLEAR(surround_dynalloc, end);
    /* This computes how much masking takes place between surround channels */
    if (!hybrid && st->energy_mask && !st->lfe) {
        int mask_end;
        int midband;
        int count_dynalloc;
        int32_t mask_avg = 0;
        int32_t diff = 0;
        int count = 0;
        mask_end = IMAX(2, st->lastCodedBands);
        for (c = 0; c < C; c++) {
            for (i = 0; i < mask_end; i++) {
                int16_t mask;
                mask = MAX16(MIN16(st->energy_mask[nbEBands * c + i], QCONST16(.25f, DB_SHIFT)),
                             -QCONST16(2.0f, DB_SHIFT));
                if (mask > 0) mask = HALF16(mask);
                mask_avg += MULT16_16(mask, eBands[i + 1] - eBands[i]);
                count += eBands[i + 1] - eBands[i];
                diff += MULT16_16(mask, 1 + 2 * i - mask_end);
            }
        }
        celt_assert(count > 0);
        mask_avg = DIV32_16(mask_avg, count);
        mask_avg += QCONST16(.2f, DB_SHIFT);
        diff = diff * 6 / (C * (mask_end - 1) * (mask_end + 1) * mask_end);
        /* Again, being conservative */
        diff = HALF32(diff);
        diff = MAX32(MIN32(diff, QCONST32(.031f, DB_SHIFT)), -QCONST32(.031f, DB_SHIFT));
        /* Find the band that's in the middle of the coded spectrum */
        for (midband = 0; eBands[midband + 1] < eBands[mask_end] / 2; midband++)
            ;
        count_dynalloc = 0;
        for (i = 0; i < mask_end; i++) {
            int32_t lin;
            int16_t unmask;
            lin = mask_avg + diff * (i - midband);
            if (C == 2)
                unmask = MAX16(st->energy_mask[i], st->energy_mask[nbEBands + i]);
            else
                unmask = st->energy_mask[i];
            unmask = MIN16(unmask, QCONST16(.0f, DB_SHIFT));
            unmask -= lin;
            if (unmask > QCONST16(.25f, DB_SHIFT)) {
                surround_dynalloc[i] = unmask - QCONST16(.25f, DB_SHIFT);
                count_dynalloc++;
            }
        }
        if (count_dynalloc >= 3) {
            /* If we need dynalloc in many bands, it's probably because our
               initial masking rate was too low. */
            mask_avg += QCONST16(.25f, DB_SHIFT);
            if (mask_avg > 0) {
                /* Something went really wrong in the original calculations,
                   disabling masking. */
                mask_avg = 0;
                diff = 0;
                OPUS_CLEAR(surround_dynalloc, mask_end);
            } else {
                for (i = 0; i < mask_end; i++)
                    surround_dynalloc[i] = MAX16(0, surround_dynalloc[i] - QCONST16(.25f, DB_SHIFT));
            }
        }
        mask_avg += QCONST16(.2f, DB_SHIFT);
        /* Convert to 1/64th units used for the trim */
        surround_trim = 64 * diff;
        /*printf("%d %d ", mask_avg, surround_trim);*/
        surround_masking = mask_avg;
    }
    /* Temporal VBR (but not for LFE) */
    if (!st->lfe) {
        int16_t follow = -QCONST16(10.0f, DB_SHIFT);
        int32_t frame_avg = 0;
        int16_t offset = shortBlocks ? HALF16(SHL16(LM, DB_SHIFT)) : 0;
        for (i = start; i < end; i++) {
            follow = MAX16(follow - QCONST16(1.f, DB_SHIFT), bandLogE[i] - offset);
            if (C == 2) follow = MAX16(follow, bandLogE[i + nbEBands] - offset);
            frame_avg += follow;
        }
        frame_avg /= (end - start);
        temporal_vbr = SUB16(frame_avg, st->spec_avg);
        temporal_vbr = MIN16(QCONST16(3.f, DB_SHIFT), MAX16(-QCONST16(1.5f, DB_SHIFT), temporal_vbr));
        st->spec_avg += MULT16_16_Q15(QCONST16(.02f, 15), temporal_vbr);
    }
    /*for (i=0;i<21;i++)
       printf("%f ", bandLogE[i]);
    printf("\n");*/

    if (!secondMdct) {
        OPUS_COPY(bandLogE2, bandLogE, C * nbEBands);
    }

    /* Last chance to catch any transient we might have missed in the
       time-domain analysis */
    if (LM > 0 && ec_tell(enc) + 3 <= total_bits && !isTransient && st->complexity >= 5 && !st->lfe && !hybrid) {
        if (patch_transient_decision(bandLogE, oldBandE, nbEBands, start, end, C)) {
            isTransient = 1;
            shortBlocks = M;
            compute_mdcts(mode, shortBlocks, in, freq, C, CC, LM, st->upsample, st->arch);
            compute_band_energies(mode, freq, bandE, effEnd, C, LM, st->arch);
            amp2Log2(mode, effEnd, end, bandE, bandLogE, C);
            /* Compensate for the scaling of short vs long mdcts */
            for (i = 0; i < C * nbEBands; i++) bandLogE2[i] += HALF16(SHL16(LM, DB_SHIFT));
            tf_estimate = QCONST16(.2f, 14);
        }
    }

    if (LM > 0 && ec_tell(enc) + 3 <= total_bits) ec_enc_bit_logp(enc, isTransient, 3);

    ALLOC(X, C * N, int16_t); /**< Interleaved normalised MDCTs */

    /* Band normalisation */
    normalise_bands(mode, freq, X, bandE, effEnd, C, M);

    enable_tf_analysis = effectiveBytes >= 15 * C && !hybrid && st->complexity >= 2 && !st->lfe;

    ALLOC(offsets, nbEBands, int);
    ALLOC(importance, nbEBands, int);
    ALLOC(spread_weight, nbEBands, int);

    maxDepth = dynalloc_analysis(bandLogE, bandLogE2, nbEBands, start, end, C, offsets, st->lsb_depth, mode->logN,
                                 isTransient, st->vbr, st->constrained_vbr, eBands, LM, effectiveBytes, &tot_boost,
                                 st->lfe, surround_dynalloc, &st->analysis, importance, spread_weight);

    ALLOC(tf_res, nbEBands, int);
    /* Disable variable tf resolution for hybrid and at very low bitrate */
    if (enable_tf_analysis) {
        int lambda;
        lambda = IMAX(80, 20480 / effectiveBytes + 2);
        tf_select = tf_analysis(mode, effEnd, isTransient, tf_res, lambda, X, N, LM, tf_estimate, tf_chan, importance);
        for (i = effEnd; i < end; i++) tf_res[i] = tf_res[effEnd - 1];
    } else if (hybrid && weak_transient) {
        /* For weak transients, we rely on the fact that improving time resolution using
           TF on a long window is imperfect and will not result in an energy collapse at
           low bitrate. */
        for (i = 0; i < end; i++) tf_res[i] = 1;
        tf_select = 0;
    } else if (hybrid && effectiveBytes < 15 && st->silk_info.signalType != 2) {
        /* For low bitrate hybrid, we force temporal resolution to 5 ms rather than 2.5 ms. */
        for (i = 0; i < end; i++) tf_res[i] = 0;
        tf_select = isTransient;
    } else {
        for (i = 0; i < end; i++) tf_res[i] = isTransient;
        tf_select = 0;
    }

    ALLOC(error, C * nbEBands, int16_t);
    c = 0;
    do {
        for (i = start; i < end; i++) {
            /* When the energy is stable, slightly bias energy quantization towards
               the previous error to make the gain more stable (a constant offset is
               better than fluctuations). */
            if (ABS32(SUB32(bandLogE[i + c * nbEBands], oldBandE[i + c * nbEBands])) < QCONST16(2.f, DB_SHIFT)) {
                bandLogE[i + c * nbEBands] -= MULT16_16_Q15(energyError[i + c * nbEBands], QCONST16(0.25f, 15));
            }
        }
    } while (++c < C);
    quant_coarse_energy(mode, start, end, effEnd, bandLogE, oldBandE, total_bits, error, enc, C, LM, nbAvailableBytes,
                        st->force_intra, &st->delayedIntra, st->complexity >= 4, st->loss_rate, st->lfe);

    tf_encode(start, end, isTransient, tf_res, LM, tf_select, enc);

    if (ec_tell(enc) + 4 <= total_bits) {
        if (st->lfe) {
            st->tapset_decision = 0;
            st->spread_decision = SPREAD_NORMAL;
        } else if (hybrid) {
            if (st->complexity == 0)
                st->spread_decision = SPREAD_NONE;
            else if (isTransient)
                st->spread_decision = SPREAD_NORMAL;
            else
                st->spread_decision = SPREAD_AGGRESSIVE;
        } else if (shortBlocks || st->complexity < 3 || nbAvailableBytes < 10 * C) {
            if (st->complexity == 0)
                st->spread_decision = SPREAD_NONE;
            else
                st->spread_decision = SPREAD_NORMAL;
        } else {
            /* Disable new spreading+tapset estimator until we can show it works
               better than the old one. So far it seems like spreading_decision()
               works best. */

            {
                st->spread_decision =
                    spreading_decision(mode, X, &st->tonal_average, st->spread_decision, &st->hf_average,
                                       &st->tapset_decision, pf_on && !shortBlocks, effEnd, C, M, spread_weight);
            }
            /*printf("%d %d\n", st->tapset_decision, st->spread_decision);*/
            /*printf("%f %d %f %d\n\n", st->analysis.tonality, st->spread_decision, st->analysis.tonality_slope,
             * st->tapset_decision);*/
        }
        ec_enc_icdf(enc, st->spread_decision, spread_icdf, 5);
    }

    /* For LFE, everything interesting is in the first band */
    if (st->lfe) offsets[0] = IMIN(8, effectiveBytes / 3);
    ALLOC(cap, nbEBands, int);
    init_caps(mode, cap, LM, C);

    dynalloc_logp = 6;
    total_bits <<= BITRES;
    total_boost = 0;
    tell = ec_tell_frac(enc);
    for (i = start; i < end; i++) {
        int width, quanta;
        int dynalloc_loop_logp;
        int boost;
        int j;
        width = C * (eBands[i + 1] - eBands[i]) << LM;
        /* quanta is 6 bits, but no more than 1 bit/sample
           and no less than 1/8 bit/sample */
        quanta = IMIN(width << BITRES, IMAX(6 << BITRES, width));
        dynalloc_loop_logp = dynalloc_logp;
        boost = 0;
        for (j = 0; tell + (dynalloc_loop_logp << BITRES) < total_bits - total_boost && boost < cap[i]; j++) {
            int flag;
            flag = j < offsets[i];
            ec_enc_bit_logp(enc, flag, dynalloc_loop_logp);
            tell = ec_tell_frac(enc);
            if (!flag) break;
            boost += quanta;
            total_boost += quanta;
            dynalloc_loop_logp = 1;
        }
        /* Making dynalloc more likely */
        if (j) dynalloc_logp = IMAX(2, dynalloc_logp - 1);
        offsets[i] = boost;
    }

    if (C == 2) {
        static const int16_t intensity_thresholds[21] =
            /* 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19  20  off*/
            {1, 2, 3, 4, 5, 6, 7, 8, 16, 24, 36, 44, 50, 56, 62, 67, 72, 79, 88, 106, 134};
        static const int16_t intensity_histeresis[21] = {1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4, 5, 6, 8, 8};

        /* Always use MS for 2.5 ms frames until we can do a better analysis */
        if (LM != 0) dual_stereo = stereo_analysis(mode, X, LM, N);

        st->intensity = hysteresis_decision((int16_t)(equiv_rate / 1000), intensity_thresholds, intensity_histeresis,
                                            21, st->intensity);
        st->intensity = IMIN(end, IMAX(start, st->intensity));
    }

    alloc_trim = 5;
    if (tell + (6 << BITRES) <= total_bits - total_boost) {
        if (start > 0 || st->lfe) {
            st->stereo_saving = 0;
            alloc_trim = 5;
        } else {
            alloc_trim = alloc_trim_analysis(mode, X, bandLogE, end, LM, C, N, &st->analysis, &st->stereo_saving,
                                             tf_estimate, st->intensity, surround_trim, equiv_rate, st->arch);
        }
        ec_enc_icdf(enc, alloc_trim, trim_icdf, 7);
        tell = ec_tell_frac(enc);
    }

    /* Variable bitrate */
    if (vbr_rate > 0) {
        int16_t alpha;
        int32_t delta;
        /* The target rate in 8th bits per frame */
        int32_t target, base_target;
        int32_t min_allowed;
        int lm_diff = mode->maxLM - LM;

        /* Don't attempt to use more than 510 kb/s, even for frames smaller than 20 ms.
           The CELT allocator will just not be able to use more than that anyway. */
        nbCompressedBytes = IMIN(nbCompressedBytes, 1275 >> (3 - LM));
        if (!hybrid) {
            base_target = vbr_rate - ((40 * C + 20) << BITRES);
        } else {
            base_target = IMAX(0, vbr_rate - ((9 * C + 4) << BITRES));
        }

        if (st->constrained_vbr) base_target += (st->vbr_offset >> lm_diff);

        if (!hybrid) {
            target = compute_vbr(mode, &st->analysis, base_target, LM, equiv_rate, st->lastCodedBands, C, st->intensity,
                                 st->constrained_vbr, st->stereo_saving, tot_boost, tf_estimate, pitch_change, maxDepth,
                                 st->lfe, st->energy_mask != NULL, surround_masking, temporal_vbr);
        } else {
            target = base_target;
            /* Tonal frames (offset<100) need more bits than noisy (offset>100) ones. */
            if (st->silk_info.offset < 100) target += 12 << BITRES >> (3 - LM);
            if (st->silk_info.offset > 100) target -= 18 << BITRES >> (3 - LM);
            /* Boosting bitrate on transients and vowels with significant temporal
               spikes. */
            target += (int32_t)MULT16_16_Q14(tf_estimate - QCONST16(.25f, 14), (50 << BITRES));
            /* If we have a strong transient, let's make sure it has enough bits to code
               the first two bands, so that it can use folding rather than noise. */
            if (tf_estimate > QCONST16(.7f, 14)) target = IMAX(target, 50 << BITRES);
        }
        /* The current offset is removed from the target and the space used
           so far is added*/
        target = target + tell;
        /* In VBR mode the frame size must not be reduced so much that it would
            result in the encoder running out of bits.
           The margin of 2 bytes ensures that none of the bust-prevention logic
            in the decoder will have triggered so far. */
        min_allowed = ((tell + total_boost + (1 << (BITRES + 3)) - 1) >> (BITRES + 3)) + 2;
        /* Take into account the 37 bits we need to have left in the packet to
           signal a redundant frame in hybrid mode. Creating a shorter packet would
           create an entropy coder desync. */
        if (hybrid)
            min_allowed = IMAX(min_allowed,
                               (tell0_frac + (37 << BITRES) + total_boost + (1 << (BITRES + 3)) - 1) >> (BITRES + 3));

        nbAvailableBytes = (target + (1 << (BITRES + 2))) >> (BITRES + 3);
        nbAvailableBytes = IMAX(min_allowed, nbAvailableBytes);
        nbAvailableBytes = IMIN(nbCompressedBytes, nbAvailableBytes);

        /* By how much did we "miss" the target on that frame */
        delta = target - vbr_rate;

        target = nbAvailableBytes << (BITRES + 3);

        /*If the frame is silent we don't adjust our drift, otherwise
          the encoder will shoot to very high rates after hitting a
          span of silence, but we do allow the bitres to refill.
          This means that we'll undershoot our target in CVBR/VBR modes
          on files with lots of silence. */
        if (silence) {
            nbAvailableBytes = 2;
            target = 2 * 8 << BITRES;
            delta = 0;
        }

        if (st->vbr_count < 970) {
            st->vbr_count++;
            alpha = celt_rcp(SHL32(EXTEND32(st->vbr_count + 20), 16));
        } else
            alpha = QCONST16(.001f, 15);
        /* How many bits have we used in excess of what we're allowed */
        if (st->constrained_vbr) st->vbr_reservoir += target - vbr_rate;
        /*printf ("%d\n", st->vbr_reservoir);*/

        /* Compute the offset we need to apply in order to reach the target */
        if (st->constrained_vbr) {
            st->vbr_drift += (int32_t)MULT16_32_Q15(alpha, (delta * (1 << lm_diff)) - st->vbr_offset - st->vbr_drift);
            st->vbr_offset = -st->vbr_drift;
        }
        /*printf ("%d\n", st->vbr_drift);*/

        if (st->constrained_vbr && st->vbr_reservoir < 0) {
            /* We're under the min value -- increase rate */
            int adjust = (-st->vbr_reservoir) / (8 << BITRES);
            /* Unless we're just coding silence */
            nbAvailableBytes += silence ? 0 : adjust;
            st->vbr_reservoir = 0;
            /*printf ("+%d\n", adjust);*/
        }
        nbCompressedBytes = IMIN(nbCompressedBytes, nbAvailableBytes);
        /*printf("%d\n", nbCompressedBytes*50*8);*/
        /* This moves the raw bits to take into account the new compressed size */
        ec_enc_shrink(enc, nbCompressedBytes);
    }

    /* Bit allocation */
    ALLOC(fine_quant, nbEBands, int);
    ALLOC(pulses, nbEBands, int);
    ALLOC(fine_priority, nbEBands, int);

    /* bits =           packet size                    - where we are - safety*/
    bits = (((int32_t)nbCompressedBytes * 8) << BITRES) - ec_tell_frac(enc) - 1;
    anti_collapse_rsv = isTransient && LM >= 2 && bits >= ((LM + 2) << BITRES) ? (1 << BITRES) : 0;
    bits -= anti_collapse_rsv;
    signalBandwidth = end - 1;

    if (st->lfe) signalBandwidth = 1;
    codedBands =
        clt_compute_allocation(mode, start, end, offsets, cap, alloc_trim, &st->intensity, &dual_stereo, bits, &balance,
                               pulses, fine_quant, fine_priority, C, LM, enc, 1, st->lastCodedBands, signalBandwidth);
    if (st->lastCodedBands)
        st->lastCodedBands = IMIN(st->lastCodedBands + 1, IMAX(st->lastCodedBands - 1, codedBands));
    else
        st->lastCodedBands = codedBands;

    quant_fine_energy(mode, start, end, oldBandE, error, fine_quant, enc, C);

    /* Residual quantisation */
    ALLOC(collapse_masks, C * nbEBands, unsigned char);
    quant_all_bands(1, mode, start, end, X, C == 2 ? X + N : NULL, collapse_masks, bandE, pulses, shortBlocks,
                    st->spread_decision, dual_stereo, st->intensity, tf_res,
                    nbCompressedBytes * (8 << BITRES) - anti_collapse_rsv, balance, enc, LM, codedBands, &st->rng,
                    st->complexity, st->arch, st->disable_inv);

    if (anti_collapse_rsv > 0) {
        anti_collapse_on = st->consec_transient < 2;

        ec_enc_bits(enc, anti_collapse_on, 1);
    }
    quant_energy_finalise(mode, start, end, oldBandE, error, fine_quant, fine_priority,
                          nbCompressedBytes * 8 - ec_tell(enc), enc, C);
    OPUS_CLEAR(energyError, nbEBands * CC);
    c = 0;
    do {
        for (i = start; i < end; i++) {
            energyError[i + c * nbEBands] =
                MAX16(-QCONST16(0.5f, 15), MIN16(QCONST16(0.5f, 15), error[i + c * nbEBands]));
        }
    } while (++c < C);

    if (silence) {
        for (i = 0; i < C * nbEBands; i++) oldBandE[i] = -QCONST16(28.f, DB_SHIFT);
    }

    st->prefilter_period = pitch_index;
    st->prefilter_gain = gain1;
    st->prefilter_tapset = prefilter_tapset;

    if (CC == 2 && C == 1) {
        OPUS_COPY(&oldBandE[nbEBands], oldBandE, nbEBands);
    }

    if (!isTransient) {
        OPUS_COPY(oldLogE2, oldLogE, CC * nbEBands);
        OPUS_COPY(oldLogE, oldBandE, CC * nbEBands);
    } else {
        for (i = 0; i < CC * nbEBands; i++) oldLogE[i] = MIN16(oldLogE[i], oldBandE[i]);
    }
    /* In case start or end were to change */
    c = 0;
    do {
        for (i = 0; i < start; i++) {
            oldBandE[c * nbEBands + i] = 0;
            oldLogE[c * nbEBands + i] = oldLogE2[c * nbEBands + i] = -QCONST16(28.f, DB_SHIFT);
        }
        for (i = end; i < nbEBands; i++) {
            oldBandE[c * nbEBands + i] = 0;
            oldLogE[c * nbEBands + i] = oldLogE2[c * nbEBands + i] = -QCONST16(28.f, DB_SHIFT);
        }
    } while (++c < CC);

    if (isTransient || transient_got_disabled)
        st->consec_transient++;
    else
        st->consec_transient = 0;
    st->rng = enc->rng;

    /* If there's any room left (can only happen for very high rates),
       it's already filled with zeros */
    ec_enc_done(enc);

    if (ec_get_error(enc))
        return OPUS_INTERNAL_ERROR;
    else
        return nbCompressedBytes;
}
//----------------------------------------------------------------------------------------------------------------------

int celt_encoder_ctl(CELTEncoder *__restrict__ st, int request, ...) {
    va_list ap;

    va_start(ap, request);
    switch (request) {
        case OPUS_SET_COMPLEXITY_REQUEST: {
            int value = va_arg(ap, int32_t);
            if (value < 0 || value > 10) goto bad_arg;
            st->complexity = value;
        } break;
        case CELT_SET_START_BAND_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 0 || value >= st->mode->nbEBands) goto bad_arg;
            st->start = value;
        } break;
        case CELT_SET_END_BAND_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 1 || value > st->mode->nbEBands) goto bad_arg;
            st->end = value;
        } break;
        case CELT_SET_PREDICTION_REQUEST: {
            int value = va_arg(ap, int32_t);
            if (value < 0 || value > 2) goto bad_arg;
            st->disable_pf = value <= 1;
            st->force_intra = value == 0;
        } break;
        case OPUS_SET_PACKET_LOSS_PERC_REQUEST: {
            int value = va_arg(ap, int32_t);
            if (value < 0 || value > 100) goto bad_arg;
            st->loss_rate = value;
        } break;
        case OPUS_SET_VBR_CONSTRAINT_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            st->constrained_vbr = value;
        } break;
        case OPUS_SET_VBR_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            st->vbr = value;
        } break;
        case OPUS_SET_BITRATE_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value <= 500 && value != OPUS_BITRATE_MAX) goto bad_arg;
            value = IMIN(value, 260000 * st->channels);
            st->bitrate = value;
        } break;
        case CELT_SET_CHANNELS_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 1 || value > 2) goto bad_arg;
            st->stream_channels = value;
        } break;
        case OPUS_SET_LSB_DEPTH_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 8 || value > 24) goto bad_arg;
            st->lsb_depth = value;
        } break;
        case OPUS_GET_LSB_DEPTH_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            *value = st->lsb_depth;
        } break;
        case OPUS_SET_PHASE_INVERSION_DISABLED_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            if (value < 0 || value > 1) {
                goto bad_arg;
            }
            st->disable_inv = value;
        } break;
        case OPUS_GET_PHASE_INVERSION_DISABLED_REQUEST: {
            int32_t *value = va_arg(ap, int32_t *);
            if (!value) {
                goto bad_arg;
            }
            *value = st->disable_inv;
        } break;
        case OPUS_RESET_STATE: {
            int i;
            int16_t *oldBandE, *oldLogE, *oldLogE2;
            oldBandE = (int16_t *)(st->in_mem + st->channels * (st->mode->overlap + COMBFILTER_MAXPERIOD));
            oldLogE = oldBandE + st->channels * st->mode->nbEBands;
            oldLogE2 = oldLogE + st->channels * st->mode->nbEBands;
            OPUS_CLEAR((char *)&st->ENCODER_RESET_START, opus_custom_encoder_get_size(st->mode, st->channels) -
                                                             ((char *)&st->ENCODER_RESET_START - (char *)st));
            for (i = 0; i < st->channels * st->mode->nbEBands; i++)
                oldLogE[i] = oldLogE2[i] = -QCONST16(28.f, DB_SHIFT);
            st->vbr_offset = 0;
            st->delayedIntra = 1;
            st->spread_decision = SPREAD_NORMAL;
            st->tonal_average = 256;
            st->hf_average = 0;
            st->tapset_decision = 0;
        } break;

        case CELT_SET_SIGNALLING_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            st->signalling = value;
        } break;
        case CELT_SET_ANALYSIS_REQUEST: {
            AnalysisInfo *info = va_arg(ap, AnalysisInfo *);
            if (info) OPUS_COPY(&st->analysis, info, 1);
        } break;
        case CELT_SET_SILK_INFO_REQUEST: {
            SILKInfo *info = va_arg(ap, SILKInfo *);
            if (info) OPUS_COPY(&st->silk_info, info, 1);
        } break;
        case CELT_GET_MODE_REQUEST: {
            const CELTMode **value = va_arg(ap, const CELTMode **);
            if (value == 0) goto bad_arg;
            *value = st->mode;
        } break;
        case OPUS_GET_FINAL_RANGE_REQUEST: {
            uint32_t *value = va_arg(ap, uint32_t *);
            if (value == 0) goto bad_arg;
            *value = st->rng;
        } break;
        case OPUS_SET_LFE_REQUEST: {
            int32_t value = va_arg(ap, int32_t);
            st->lfe = value;
        } break;
        case OPUS_SET_ENERGY_MASK_REQUEST: {
            int16_t *value = va_arg(ap, int16_t *);
            st->energy_mask = value;
        } break;
        default:
            goto bad_request;
    }
    va_end(ap);
    return OPUS_OK;
bad_arg:
    va_end(ap);
    return OPUS_BAD_ARG;
bad_request:
    va_end(ap);
    return OPUS_UNIMPLEMENTED;
}
//----------------------------------------------------------------------------------------------------------------------

void _celt_lpc(int16_t *_lpc,     /* out: [0...p-1] LPC coefficients      */
               const int32_t *ac, /* in:  [0...p] autocorrelation values  */
               int p) {
    int i, j;
    int32_t r;
    int32_t error = ac[0];
    int32_t lpc[LPC_ORDER];
    OPUS_CLEAR(lpc, p);
    if (ac[0] != 0) {
        for (i = 0; i < p; i++) {
            /* Sum up this iteration's reflection coefficient */
            int32_t rr = 0;
            for (j = 0; j < i; j++) rr += MULT32_32_Q31(lpc[j], ac[i - j]);
            rr += SHR32(ac[i + 1], 3);
            r = -frac_div32(SHL32(rr, 3), error);
            /*  Update LPC coefficients and total error */
            lpc[i] = SHR32(r, 3);
            for (j = 0; j < (i + 1) >> 1; j++) {
                int32_t tmp1, tmp2;
                tmp1 = lpc[j];
                tmp2 = lpc[i - 1 - j];
                lpc[j] = tmp1 + MULT32_32_Q31(r, tmp2);
                lpc[i - 1 - j] = tmp2 + MULT32_32_Q31(r, tmp1);
            }

            error = error - MULT32_32_Q31(MULT32_32_Q31(r, r), error);
            /* Bail out once we get 30 dB gain */
            if (error < SHR32(ac[0], 10)) break;
        }
    }
    for (i = 0; i < p; i++) _lpc[i] = ROUND16(lpc[i], 16);
}
//----------------------------------------------------------------------------------------------------------------------

void celt_fir_c(const int16_t *x, const int16_t *num, int16_t *y, int N, int ord, int arch) {
    int i, j;
    VARDECL(int16_t, rnum);
    SAVE_STACK;
    celt_assert(x != y);
    ALLOC(rnum, ord, int16_t);
    for (i = 0; i < ord; i++) rnum[i] = num[ord - i - 1];
    for (i = 0; i < N - 3; i += 4) {
        int32_t sum[4];
        sum[0] = SHL32(EXTEND32(x[i]), 12);
        sum[1] = SHL32(EXTEND32(x[i + 1]), 12);
        sum[2] = SHL32(EXTEND32(x[i + 2]), 12);
        sum[3] = SHL32(EXTEND32(x[i + 3]), 12);
        xcorr_kernel(rnum, x + i - ord, sum, ord, arch);
        y[i] = ROUND16(sum[0], 12);
        y[i + 1] = ROUND16(sum[1], 12);
        y[i + 2] = ROUND16(sum[2], 12);
        y[i + 3] = ROUND16(sum[3], 12);
    }
    for (; i < N; i++) {
        int32_t sum = SHL32(EXTEND32(x[i]), 12);
        for (j = 0; j < ord; j++) sum = MAC16_16(sum, rnum[j], x[i + j - ord]);
        y[i] = ROUND16(sum, 12);
    }
}
//----------------------------------------------------------------------------------------------------------------------

void celt_iir(const int32_t *_x, const int16_t *den, int32_t *_y, int N, int ord, int16_t *mem, int arch) {
    int i, j;
    VARDECL(int16_t, rden);
    VARDECL(int16_t, y);
    SAVE_STACK;

    celt_assert((ord & 3) == 0);
    ALLOC(rden, ord, int16_t);
    ALLOC(y, N + ord, int16_t);
    for (i = 0; i < ord; i++) rden[i] = den[ord - i - 1];
    for (i = 0; i < ord; i++) y[i] = -mem[ord - i - 1];
    for (; i < N + ord; i++) y[i] = 0;
    for (i = 0; i < N - 3; i += 4) {
        /* Unroll by 4 as if it were an FIR filter */
        int32_t sum[4];
        sum[0] = _x[i];
        sum[1] = _x[i + 1];
        sum[2] = _x[i + 2];
        sum[3] = _x[i + 3];
        xcorr_kernel(rden, y + i, sum, ord, arch);

        /* Patch up the result to compensate for the fact that this is an IIR */
        y[i + ord] = -SROUND16(sum[0], 12);
        _y[i] = sum[0];
        sum[1] = MAC16_16(sum[1], y[i + ord], den[0]);
        y[i + ord + 1] = -SROUND16(sum[1], 12);
        _y[i + 1] = sum[1];
        sum[2] = MAC16_16(sum[2], y[i + ord + 1], den[0]);
        sum[2] = MAC16_16(sum[2], y[i + ord], den[1]);
        y[i + ord + 2] = -SROUND16(sum[2], 12);
        _y[i + 2] = sum[2];

        sum[3] = MAC16_16(sum[3], y[i + ord + 2], den[0]);
        sum[3] = MAC16_16(sum[3], y[i + ord + 1], den[1]);
        sum[3] = MAC16_16(sum[3], y[i + ord], den[2]);
        y[i + ord + 3] = -SROUND16(sum[3], 12);
        _y[i + 3] = sum[3];
    }
    for (; i < N; i++) {
        int32_t sum = _x[i];
        for (j = 0; j < ord; j++) sum -= MULT16_16(rden[j], y[i + j]);
        y[i + ord] = SROUND16(sum, 12);
        _y[i] = sum;
    }
    for (i = 0; i < ord; i++) mem[i] = _y[N - i - 1];
}
//----------------------------------------------------------------------------------------------------------------------

int _celt_autocorr(const int16_t *x, /*  in: [0...n-1] samples x   */
                   int32_t *ac,      /* out: [0...lag-1] ac values */
                   const int16_t *window, int overlap, int lag, int n, int arch) {
    int32_t d;
    int i, k;
    int fastN = n - lag;
    int shift;
    const int16_t *xptr;
    VARDECL(int16_t, xx);
    SAVE_STACK;
    ALLOC(xx, n, int16_t);
    celt_assert(n > 0);
    celt_assert(overlap >= 0);
    if (overlap == 0) {
        xptr = x;
    } else {
        for (i = 0; i < n; i++) xx[i] = x[i];
        for (i = 0; i < overlap; i++) {
            xx[i] = MULT16_16_Q15(x[i], window[i]);
            xx[n - i - 1] = MULT16_16_Q15(x[n - i - 1], window[i]);
        }
        xptr = xx;
    }
    shift = 0;
    {
        int32_t ac0;
        ac0 = 1 + (n << 7);
        if (n & 1) ac0 += SHR32(MULT16_16(xptr[0], xptr[0]), 9);
        for (i = (n & 1); i < n; i += 2) {
            ac0 += SHR32(MULT16_16(xptr[i], xptr[i]), 9);
            ac0 += SHR32(MULT16_16(xptr[i + 1], xptr[i + 1]), 9);
        }

        shift = celt_ilog2(ac0) - 30 + 10;
        shift = (shift) / 2;
        if (shift > 0) {
            for (i = 0; i < n; i++) xx[i] = PSHR32(xptr[i], shift);
            xptr = xx;
        } else
            shift = 0;
    }
    celt_pitch_xcorr(xptr, xptr, ac, fastN, lag + 1, arch);
    for (k = 0; k <= lag; k++) {
        for (i = k + fastN, d = 0; i < n; i++) d = MAC16_16(d, xptr[i], xptr[i - k]);
        ac[k] += d;
    }
    shift = 2 * shift;
    if (shift <= 0) ac[0] += SHL32((int32_t)1, -shift);
    if (ac[0] < 268435456) {
        int shift2 = 29 - EC_ILOG(ac[0]);
        for (i = 0; i <= lag; i++) ac[i] = SHL32(ac[i], shift2);
        shift -= shift2;
    } else if (ac[0] >= 536870912) {
        int shift2 = 1;
        if (ac[0] >= 1073741824) shift2++;
        for (i = 0; i <= lag; i++) ac[i] = SHR32(ac[i], shift2);
        shift += shift2;
    }

    return shift;
}
//----------------------------------------------------------------------------------------------------------------------

static uint32_t icwrs(int _n, const int *_y) {
    uint32_t i;
    int j;
    int k;
    celt_assert(_n >= 2);
    j = _n - 1;
    i = _y[j] < 0;
    k = abs(_y[j]);
    do {
        j--;
        i += CELT_PVQ_U(_n - j, k);
        k += abs(_y[j]);
        if (_y[j] < 0) i += CELT_PVQ_U(_n - j, k + 1);
    } while (j > 0);
    return i;
}
//----------------------------------------------------------------------------------------------------------------------

void encode_pulses(const int *_y, int _n, int _k, ec_enc *_enc) {
    celt_assert(_k > 0);
    ec_enc_uint(_enc, icwrs(_n, _y), CELT_PVQ_V(_n, _k));
}
//----------------------------------------------------------------------------------------------------------------------

static int32_t cwrsi(int _n, int _k, uint32_t _i, int *_y) {
    uint32_t p;
    int s;
    int k0;
    int16_t val;
    int32_t yy = 0;
    celt_assert(_k > 0);
    celt_assert(_n > 1);
    while (_n > 2) {
        uint32_t q;
        /*Lots of pulses case:*/
        if (_k >= _n) {
            const uint32_t *row;
            row = CELT_PVQ_U_ROW[_n];
            /*Are the pulses in this dimension negative?*/
            p = row[_k + 1];
            s = -(_i >= p);
            _i -= p & s;
            /*Count how many pulses were placed in this dimension.*/
            k0 = _k;
            q = row[_n];
            if (q > _i) {
                assert(p > q);
                _k = _n;
                do p = CELT_PVQ_U_ROW[--_k][_n];
                while (p > _i);
            } else
                for (p = row[_k]; p > _i; p = row[_k]) _k--;
            _i -= p;
            val = (k0 - _k + s) ^ s;
            *_y++ = val;
            yy = MAC16_16(yy, val, val);
        }
        /*Lots of dimensions case:*/
        else {
            /*Are there any pulses in this dimension at all?*/
            p = CELT_PVQ_U_ROW[_k][_n];
            q = CELT_PVQ_U_ROW[_k + 1][_n];
            if (p <= _i && _i < q) {
                _i -= p;
                *_y++ = 0;
            } else {
                /*Are the pulses in this dimension negative?*/
                s = -(_i >= q);
                _i -= q & s;
                /*Count how many pulses were placed in this dimension.*/
                k0 = _k;
                do p = CELT_PVQ_U_ROW[--_k][_n];
                while (p > _i);
                _i -= p;
                val = (k0 - _k + s) ^ s;
                *_y++ = val;
                yy = MAC16_16(yy, val, val);
            }
        }
        _n--;
    }
    /*_n==2*/
    p = 2 * _k + 1;
    s = -(_i >= p);
    _i -= p & s;
    k0 = _k;
    _k = (_i + 1) >> 1;
    if (_k) _i -= 2 * _k - 1;
    val = (k0 - _k + s) ^ s;
    *_y++ = val;
    yy = MAC16_16(yy, val, val);
    /*_n==1*/
    s = -(int)_i;
    val = (_k + s) ^ s;
    *_y = val;
    yy = MAC16_16(yy, val, val);
    return yy;
}
//----------------------------------------------------------------------------------------------------------------------

int32_t decode_pulses(int *_y, int _n, int _k, ec_dec *_dec) {
    return cwrsi(_n, _k, ec_dec_uint(_dec, CELT_PVQ_V(_n, _k)), _y);
}
//----------------------------------------------------------------------------------------------------------------------

/* This is a faster version of ec_tell_frac() that takes advantage of the low (1/8 bit) resolution to use just a linear
   function followed by a lookup to determine the exact transition thresholds. */
uint32_t ec_tell_frac(ec_ctx *_this) {
    static const unsigned correction[8] = {35733, 38967, 42495, 46340, 50535, 55109, 60097, 65535};
    uint32_t nbits;
    uint32_t r;
    int l;
    unsigned b;
    nbits = _this->nbits_total << BITRES;
    l = EC_ILOG(_this->rng);
    r = _this->rng >> (l - 16);
    b = (r >> 12) - 8;
    b += r > correction[b];
    l = (l << 3) + b;
    return nbits - l;
}
//----------------------------------------------------------------------------------------------------------------------

static int ec_read_byte(ec_dec *_this) { return _this->offs < _this->storage ? _this->buf[_this->offs++] : 0; }

//----------------------------------------------------------------------------------------------------------------------

static int ec_read_byte_from_end(ec_dec *_this) {
    return _this->end_offs < _this->storage ? _this->buf[_this->storage - ++(_this->end_offs)] : 0;
}
//----------------------------------------------------------------------------------------------------------------------

/*Normalizes the contents of val and rng so that rng lies entirely in the high-order symbol.*/
static void ec_dec_normalize(ec_dec *_this) {
    /*If the range is too small, rescale it and input some bits.*/
    while (_this->rng <= EC_CODE_BOT) {
        int sym;
        _this->nbits_total += EC_SYM_BITS;
        _this->rng <<= EC_SYM_BITS;
        /*Use up the remaining bits from our last symbol.*/
        sym = _this->rem;
        /*Read the next value from the input.*/
        _this->rem = ec_read_byte(_this);
        /*Take the rest of the bits we need from this new symbol.*/
        sym = (sym << EC_SYM_BITS | _this->rem) >> (EC_SYM_BITS - EC_CODE_EXTRA);
        /*And subtract them from val, capped to be less than EC_CODE_TOP.*/
        _this->val = ((_this->val << EC_SYM_BITS) + (EC_SYM_MAX & ~sym)) & (EC_CODE_TOP - 1);
    }
}
//----------------------------------------------------------------------------------------------------------------------

void ec_dec_init(ec_dec *_this, unsigned char *_buf, uint32_t _storage) {
    _this->buf = _buf;
    _this->storage = _storage;
    _this->end_offs = 0;
    _this->end_window = 0;
    _this->nend_bits = 0;
    /*This is the offset from which ec_tell() will subtract partial bits.
      The final value after the ec_dec_normalize() call will be the same as in
       the encoder, but we have to compensate for the bits that are added there.*/
    _this->nbits_total = EC_CODE_BITS + 1 - ((EC_CODE_BITS - EC_CODE_EXTRA) / EC_SYM_BITS) * EC_SYM_BITS;
    _this->offs = 0;
    _this->rng = 1U << EC_CODE_EXTRA;
    _this->rem = ec_read_byte(_this);
    _this->val = _this->rng - 1 - (_this->rem >> (EC_SYM_BITS - EC_CODE_EXTRA));
    _this->error = 0;
    /*Normalize the interval.*/
    ec_dec_normalize(_this);
}
//----------------------------------------------------------------------------------------------------------------------

unsigned ec_decode(ec_dec *_this, unsigned _ft) {
    unsigned s;
    _this->ext = celt_udiv(_this->rng, _ft);
    s = (unsigned)(_this->val / _this->ext);
    return _ft - EC_MINI(s + 1, _ft);
}
//----------------------------------------------------------------------------------------------------------------------

unsigned ec_decode_bin(ec_dec *_this, unsigned _bits) {
    unsigned s;
    _this->ext = _this->rng >> _bits;
    s = (unsigned)(_this->val / _this->ext);
    return (1U << _bits) - EC_MINI(s + 1U, 1U << _bits);
}
//----------------------------------------------------------------------------------------------------------------------

void ec_dec_update(ec_dec *_this, unsigned _fl, unsigned _fh, unsigned _ft) {
    uint32_t s;
    s = IMUL32(_this->ext, _ft - _fh);
    _this->val -= s;
    _this->rng = _fl > 0 ? IMUL32(_this->ext, _fh - _fl) : _this->rng - s;
    ec_dec_normalize(_this);
}
//----------------------------------------------------------------------------------------------------------------------

/*The probability of having a "one" is 1/(1<<_logp).*/
int ec_dec_bit_logp(ec_dec *_this, unsigned _logp) {
    uint32_t r;
    uint32_t d;
    uint32_t s;
    int ret;
    r = _this->rng;
    d = _this->val;
    s = r >> _logp;
    ret = d < s;
    if (!ret) _this->val = d - s;
    _this->rng = ret ? s : r - s;
    ec_dec_normalize(_this);
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

int ec_dec_icdf(ec_dec *_this, const unsigned char *_icdf, unsigned _ftb) {
    uint32_t r;
    uint32_t d;
    uint32_t s;
    uint32_t t;
    int ret;
    s = _this->rng;
    d = _this->val;
    r = s >> _ftb;
    ret = -1;
    do {
        t = s;
        s = IMUL32(r, _icdf[++ret]);
    } while (d < s);
    _this->val = d - s;
    _this->rng = t - s;
    ec_dec_normalize(_this);
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

uint32_t ec_dec_uint(ec_dec *_this, uint32_t _ft) {
    unsigned ft;
    unsigned s;
    int ftb;
    /*In order to optimize EC_ILOG(), it is undefined for the value 0.*/
    celt_assert(_ft > 1);
    _ft--;
    ftb = EC_ILOG(_ft);
    if (ftb > EC_UINT_BITS) {
        uint32_t t;
        ftb -= EC_UINT_BITS;
        ft = (unsigned)(_ft >> ftb) + 1;
        s = ec_decode(_this, ft);
        ec_dec_update(_this, s, s + 1, ft);
        t = (uint32_t)s << ftb | ec_dec_bits(_this, ftb);
        if (t <= _ft) return t;
        _this->error = 1;
        return _ft;
    } else {
        _ft++;
        s = ec_decode(_this, (unsigned)_ft);
        ec_dec_update(_this, s, s + 1, (unsigned)_ft);
        return s;
    }
}
//----------------------------------------------------------------------------------------------------------------------

uint32_t ec_dec_bits(ec_dec *_this, unsigned _bits) {
    ec_window window;
    int available;
    uint32_t ret;
    window = _this->end_window;
    available = _this->nend_bits;
    if ((unsigned)available < _bits) {
        do {
            window |= (ec_window)ec_read_byte_from_end(_this) << available;
            available += EC_SYM_BITS;
        } while (available <= EC_WINDOW_SIZE - EC_SYM_BITS);
    }
    ret = (uint32_t)window & (((uint32_t)1 << _bits) - 1U);
    window >>= _bits;
    available -= _bits;
    _this->end_window = window;
    _this->nend_bits = available;
    _this->nbits_total += _bits;
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------

static int ec_write_byte(ec_enc *_this, unsigned _value) {
    if (_this->offs + _this->end_offs >= _this->storage) return -1;
    _this->buf[_this->offs++] = (unsigned char)_value;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------

static int ec_write_byte_at_end(ec_enc *_this, unsigned _value) {
    if (_this->offs + _this->end_offs >= _this->storage) return -1;
    _this->buf[_this->storage - ++(_this->end_offs)] = (unsigned char)_value;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------

/*Outputs a symbol, with a carry bit. If there is a potential to propagate a carry over several symbols, they are
  buffered until it can be determined whether or not an actual carry will occur. If the counter for the buffered symbols
  overflows, then the stream becomes undecodable. This gives a theoretical limit of a few billion symbols in a single
  packet on 32-bit systems. The alternative is to truncate the range in order to force a carry, but requires similar
  carry tracking in the decoder, needlessly slowing it down.*/
static void ec_enc_carry_out(ec_enc *_this, int _c) {
    if (_c != EC_SYM_MAX) {
        /*No further carry propagation possible, flush buffer.*/
        int carry;
        carry = _c >> EC_SYM_BITS;
        /*Don't output a byte on the first write.
          This compare should be taken care of by branch-prediction thereafter.*/
        if (_this->rem >= 0) _this->error |= ec_write_byte(_this, _this->rem + carry);
        if (_this->ext > 0) {
            unsigned sym;
            sym = (EC_SYM_MAX + carry) & EC_SYM_MAX;
            do _this->error |= ec_write_byte(_this, sym);
            while (--(_this->ext) > 0);
        }
        _this->rem = _c & EC_SYM_MAX;
    } else
        _this->ext++;
}
//----------------------------------------------------------------------------------------------------------------------

static inline void ec_enc_normalize(ec_enc *_this) {
    /*If the range is too small, output some bits and rescale it.*/
    while (_this->rng <= EC_CODE_BOT) {
        ec_enc_carry_out(_this, (int)(_this->val >> EC_CODE_SHIFT));
        /*Move the next-to-high-order symbol into the high-order position.*/
        _this->val = (_this->val << EC_SYM_BITS) & (EC_CODE_TOP - 1);
        _this->rng <<= EC_SYM_BITS;
        _this->nbits_total += EC_SYM_BITS;
    }
}
//----------------------------------------------------------------------------------------------------------------------

void ec_enc_init(ec_enc *_this, unsigned char *_buf, uint32_t _size) {
    _this->buf = _buf;
    _this->end_offs = 0;
    _this->end_window = 0;
    _this->nend_bits = 0;
    /*This is the offset from which ec_tell() will subtract partial bits.*/
    _this->nbits_total = EC_CODE_BITS + 1;
    _this->offs = 0;
    _this->rng = EC_CODE_TOP;
    _this->rem = -1;
    _this->val = 0;
    _this->ext = 0;
    _this->storage = _size;
    _this->error = 0;
}
//----------------------------------------------------------------------------------------------------------------------

void ec_encode(ec_enc *_this, unsigned _fl, unsigned _fh, unsigned _ft) {
    uint32_t r;
    r = celt_udiv(_this->rng, _ft);
    if (_fl > 0) {
        _this->val += _this->rng - IMUL32(r, (_ft - _fl));
        _this->rng = IMUL32(r, (_fh - _fl));
    } else
        _this->rng -= IMUL32(r, (_ft - _fh));
    ec_enc_normalize(_this);
}
//----------------------------------------------------------------------------------------------------------------------

void ec_encode_bin(ec_enc *_this, unsigned _fl, unsigned _fh, unsigned _bits) {
    uint32_t r;
    r = _this->rng >> _bits;
    if (_fl > 0) {
        _this->val += _this->rng - IMUL32(r, ((1U << _bits) - _fl));
        _this->rng = IMUL32(r, (_fh - _fl));
    } else
        _this->rng -= IMUL32(r, ((1U << _bits) - _fh));
    ec_enc_normalize(_this);
}
//----------------------------------------------------------------------------------------------------------------------

/*The probability of having a "one" is 1/(1<<_logp).*/
void ec_enc_bit_logp(ec_enc *_this, int _val, unsigned _logp) {
    uint32_t r;
    uint32_t s;
    uint32_t l;
    r = _this->rng;
    l = _this->val;
    s = r >> _logp;
    r -= s;
    if (_val) _this->val = l + r;
    _this->rng = _val ? s : r;
    ec_enc_normalize(_this);
}
//----------------------------------------------------------------------------------------------------------------------

void ec_enc_icdf(ec_enc *_this, int _s, const unsigned char *_icdf, unsigned _ftb) {
    uint32_t r;
    r = _this->rng >> _ftb;
    if (_s > 0) {
        _this->val += _this->rng - IMUL32(r, _icdf[_s - 1]);
        _this->rng = IMUL32(r, _icdf[_s - 1] - _icdf[_s]);
    } else
        _this->rng -= IMUL32(r, _icdf[_s]);
    ec_enc_normalize(_this);
}
//----------------------------------------------------------------------------------------------------------------------

void ec_enc_uint(ec_enc *_this, uint32_t _fl, uint32_t _ft) {
    unsigned ft;
    unsigned fl;
    int ftb;
    /*In order to optimize EC_ILOG(), it is undefined for the value 0.*/
    celt_assert(_ft > 1);
    _ft--;
    ftb = EC_ILOG(_ft);
    if (ftb > EC_UINT_BITS) {
        ftb -= EC_UINT_BITS;
        ft = (_ft >> ftb) + 1;
        fl = (unsigned)(_fl >> ftb);
        ec_encode(_this, fl, fl + 1, ft);
        ec_enc_bits(_this, _fl & (((uint32_t)1 << ftb) - 1U), ftb);
    } else
        ec_encode(_this, _fl, _fl + 1, _ft + 1);
}
//----------------------------------------------------------------------------------------------------------------------

void ec_enc_bits(ec_enc *_this, uint32_t _fl, unsigned _bits) {
    ec_window window;
    int used;
    window = _this->end_window;
    used = _this->nend_bits;
    celt_assert(_bits > 0);
    if (used + _bits > EC_WINDOW_SIZE) {
        do {
            _this->error |= ec_write_byte_at_end(_this, (unsigned)window & EC_SYM_MAX);
            window >>= EC_SYM_BITS;
            used -= EC_SYM_BITS;
        } while (used >= EC_SYM_BITS);
    }
    window |= (ec_window)_fl << used;
    used += _bits;
    _this->end_window = window;
    _this->nend_bits = used;
    _this->nbits_total += _bits;
}
//----------------------------------------------------------------------------------------------------------------------

static void kf_bfly2(kiss_fft_cpx *Fout, int m, int N) {
    kiss_fft_cpx *Fout2;
    int i;
    (void)m;

    {
        int16_t tw;
        tw = QCONST16(0.7071067812f, 15);
        /* We know that m==4 here because the radix-2 is just after a radix-4 */
        celt_assert(m == 4);
        for (i = 0; i < N; i++) {
            kiss_fft_cpx t;
            Fout2 = Fout + 4;
            t = Fout2[0];
            C_SUB(Fout2[0], Fout[0], t);
            C_ADDTO(Fout[0], t);

            t.r = S_MUL(ADD32_ovflw(Fout2[1].r, Fout2[1].i), tw);
            t.i = S_MUL(SUB32_ovflw(Fout2[1].i, Fout2[1].r), tw);
            C_SUB(Fout2[1], Fout[1], t);
            C_ADDTO(Fout[1], t);

            t.r = Fout2[2].i;
            t.i = -Fout2[2].r;
            C_SUB(Fout2[2], Fout[2], t);
            C_ADDTO(Fout[2], t);

            t.r = S_MUL(SUB32_ovflw(Fout2[3].i, Fout2[3].r), tw);
            t.i = S_MUL(NEG32_ovflw(ADD32_ovflw(Fout2[3].i, Fout2[3].r)), tw);
            C_SUB(Fout2[3], Fout[3], t);
            C_ADDTO(Fout[3], t);
            Fout += 8;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------

static void kf_bfly4(kiss_fft_cpx *Fout, const size_t fstride, const kiss_fft_state *st, int m, int N, int mm) {
    int i;

    if (m == 1) {
        /* Degenerate case where all the twiddles are 1. */
        for (i = 0; i < N; i++) {
            kiss_fft_cpx scratch0, scratch1;

            C_SUB(scratch0, *Fout, Fout[2]);
            C_ADDTO(*Fout, Fout[2]);
            C_ADD(scratch1, Fout[1], Fout[3]);
            C_SUB(Fout[2], *Fout, scratch1);
            C_ADDTO(*Fout, scratch1);
            C_SUB(scratch1, Fout[1], Fout[3]);

            Fout[1].r = ADD32_ovflw(scratch0.r, scratch1.i);
            Fout[1].i = SUB32_ovflw(scratch0.i, scratch1.r);
            Fout[3].r = SUB32_ovflw(scratch0.r, scratch1.i);
            Fout[3].i = ADD32_ovflw(scratch0.i, scratch1.r);
            Fout += 4;
        }
    } else {
        int j;
        kiss_fft_cpx scratch[6];
        const kiss_twiddle_cpx *tw1, *tw2, *tw3;
        const int m2 = 2 * m;
        const int m3 = 3 * m;
        kiss_fft_cpx *Fout_beg = Fout;
        for (i = 0; i < N; i++) {
            Fout = Fout_beg + i * mm;
            tw3 = tw2 = tw1 = st->twiddles;
            /* m is guaranteed to be a multiple of 4. */
            for (j = 0; j < m; j++) {
                C_MUL(scratch[0], Fout[m], *tw1);
                C_MUL(scratch[1], Fout[m2], *tw2);
                C_MUL(scratch[2], Fout[m3], *tw3);

                C_SUB(scratch[5], *Fout, scratch[1]);
                C_ADDTO(*Fout, scratch[1]);
                C_ADD(scratch[3], scratch[0], scratch[2]);
                C_SUB(scratch[4], scratch[0], scratch[2]);
                C_SUB(Fout[m2], *Fout, scratch[3]);
                tw1 += fstride;
                tw2 += fstride * 2;
                tw3 += fstride * 3;
                C_ADDTO(*Fout, scratch[3]);

                Fout[m].r = ADD32_ovflw(scratch[5].r, scratch[4].i);
                Fout[m].i = SUB32_ovflw(scratch[5].i, scratch[4].r);
                Fout[m3].r = SUB32_ovflw(scratch[5].r, scratch[4].i);
                Fout[m3].i = ADD32_ovflw(scratch[5].i, scratch[4].r);
                ++Fout;
            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------

static void kf_bfly3(kiss_fft_cpx *Fout, const size_t fstride, const kiss_fft_state *st, int m, int N, int mm) {
    int i;
    size_t k;
    const size_t m2 = 2 * m;
    const kiss_twiddle_cpx *tw1, *tw2;
    kiss_fft_cpx scratch[5];
    kiss_twiddle_cpx epi3;

    kiss_fft_cpx *Fout_beg = Fout;
    /*epi3.r = -16384;*/ /* Unused */
    epi3.i = -28378;
    for (i = 0; i < N; i++) {
        Fout = Fout_beg + i * mm;
        tw1 = tw2 = st->twiddles;
        /* For non-custom modes, m is guaranteed to be a multiple of 4. */
        k = m;
        do {
            C_MUL(scratch[1], Fout[m], *tw1);
            C_MUL(scratch[2], Fout[m2], *tw2);

            C_ADD(scratch[3], scratch[1], scratch[2]);
            C_SUB(scratch[0], scratch[1], scratch[2]);
            tw1 += fstride;
            tw2 += fstride * 2;

            Fout[m].r = SUB32_ovflw(Fout->r, HALF_OF(scratch[3].r));
            Fout[m].i = SUB32_ovflw(Fout->i, HALF_OF(scratch[3].i));

            C_MULBYSCALAR(scratch[0], epi3.i);

            C_ADDTO(*Fout, scratch[3]);

            Fout[m2].r = ADD32_ovflw(Fout[m].r, scratch[0].i);
            Fout[m2].i = SUB32_ovflw(Fout[m].i, scratch[0].r);

            Fout[m].r = SUB32_ovflw(Fout[m].r, scratch[0].i);
            Fout[m].i = ADD32_ovflw(Fout[m].i, scratch[0].r);

            ++Fout;
        } while (--k);
    }
}
//----------------------------------------------------------------------------------------------------------------------

static void kf_bfly5(kiss_fft_cpx *Fout, const size_t fstride, const kiss_fft_state *st, int m, int N, int mm) {
    kiss_fft_cpx *Fout0, *Fout1, *Fout2, *Fout3, *Fout4;
    int i, u;
    kiss_fft_cpx scratch[13];
    const kiss_twiddle_cpx *tw;
    kiss_twiddle_cpx ya, yb;
    kiss_fft_cpx *Fout_beg = Fout;

    ya.r = 10126;
    ya.i = -31164;
    yb.r = -26510;
    yb.i = -19261;
    tw = st->twiddles;

    for (i = 0; i < N; i++) {
        Fout = Fout_beg + i * mm;
        Fout0 = Fout;
        Fout1 = Fout0 + m;
        Fout2 = Fout0 + 2 * m;
        Fout3 = Fout0 + 3 * m;
        Fout4 = Fout0 + 4 * m;

        /* For non-custom modes, m is guaranteed to be a multiple of 4. */
        for (u = 0; u < m; ++u) {
            scratch[0] = *Fout0;

            C_MUL(scratch[1], *Fout1, tw[u * fstride]);
            C_MUL(scratch[2], *Fout2, tw[2 * u * fstride]);
            C_MUL(scratch[3], *Fout3, tw[3 * u * fstride]);
            C_MUL(scratch[4], *Fout4, tw[4 * u * fstride]);

            C_ADD(scratch[7], scratch[1], scratch[4]);
            C_SUB(scratch[10], scratch[1], scratch[4]);
            C_ADD(scratch[8], scratch[2], scratch[3]);
            C_SUB(scratch[9], scratch[2], scratch[3]);

            Fout0->r = ADD32_ovflw(Fout0->r, ADD32_ovflw(scratch[7].r, scratch[8].r));
            Fout0->i = ADD32_ovflw(Fout0->i, ADD32_ovflw(scratch[7].i, scratch[8].i));

            scratch[5].r = ADD32_ovflw(scratch[0].r, ADD32_ovflw(S_MUL(scratch[7].r, ya.r), S_MUL(scratch[8].r, yb.r)));
            scratch[5].i = ADD32_ovflw(scratch[0].i, ADD32_ovflw(S_MUL(scratch[7].i, ya.r), S_MUL(scratch[8].i, yb.r)));

            scratch[6].r = ADD32_ovflw(S_MUL(scratch[10].i, ya.i), S_MUL(scratch[9].i, yb.i));
            scratch[6].i = NEG32_ovflw(ADD32_ovflw(S_MUL(scratch[10].r, ya.i), S_MUL(scratch[9].r, yb.i)));

            C_SUB(*Fout1, scratch[5], scratch[6]);
            C_ADD(*Fout4, scratch[5], scratch[6]);

            scratch[11].r =
                ADD32_ovflw(scratch[0].r, ADD32_ovflw(S_MUL(scratch[7].r, yb.r), S_MUL(scratch[8].r, ya.r)));
            scratch[11].i =
                ADD32_ovflw(scratch[0].i, ADD32_ovflw(S_MUL(scratch[7].i, yb.r), S_MUL(scratch[8].i, ya.r)));
            scratch[12].r = SUB32_ovflw(S_MUL(scratch[9].i, ya.i), S_MUL(scratch[10].i, yb.i));
            scratch[12].i = SUB32_ovflw(S_MUL(scratch[10].r, yb.i), S_MUL(scratch[9].r, ya.i));

            C_ADD(*Fout2, scratch[11], scratch[12]);
            C_SUB(*Fout3, scratch[11], scratch[12]);

            ++Fout0;
            ++Fout1;
            ++Fout2;
            ++Fout3;
            ++Fout4;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------

void opus_fft_impl(const kiss_fft_state *st, kiss_fft_cpx *fout) {
    int m2, m;
    int p;
    int L;
    int fstride[MAXFACTORS];
    int i;
    int shift;

    /* st->shift can be -1 */
    shift = st->shift > 0 ? st->shift : 0;

    fstride[0] = 1;
    L = 0;
    do {
        p = st->factors[2 * L];
        m = st->factors[2 * L + 1];
        fstride[L + 1] = fstride[L] * p;
        L++;
    } while (m != 1);
    m = st->factors[2 * L - 1];
    for (i = L - 1; i >= 0; i--) {
        if (i != 0)
            m2 = st->factors[2 * i - 1];
        else
            m2 = 1;
        switch (st->factors[2 * i]) {
            case 2:
                kf_bfly2(fout, m, fstride[i]);
                break;
            case 4:
                kf_bfly4(fout, fstride[i] << shift, st, m, fstride[i], m2);
                break;
            case 3:
                kf_bfly3(fout, fstride[i] << shift, st, m, fstride[i], m2);
                break;
            case 5:
                kf_bfly5(fout, fstride[i] << shift, st, m, fstride[i], m2);
                break;
        }
        m = m2;
    }
}
//----------------------------------------------------------------------------------------------------------------------

void opus_fft_c(const kiss_fft_state *st, const kiss_fft_cpx *fin, kiss_fft_cpx *fout) {
    int i;
    int16_t scale;
    /* Allows us to scale with MULT16_32_Q16(), which is faster than
       MULT16_32_Q15() on ARM. */
    int scale_shift = st->scale_shift - 1;
    scale = st->scale;

    celt_assert2(fin != fout, "In-place FFT not supported");
    /* Bit-reverse the input */
    for (i = 0; i < st->nfft; i++) {
        kiss_fft_cpx x = fin[i];
        fout[st->bitrev[i]].r = SHR32(MULT16_32_Q16(scale, x.r), scale_shift);
        fout[st->bitrev[i]].i = SHR32(MULT16_32_Q16(scale, x.i), scale_shift);
    }
    opus_fft_impl(st, fout);
}
//----------------------------------------------------------------------------------------------------------------------

void opus_ifft_c(const kiss_fft_state *st, const kiss_fft_cpx *fin, kiss_fft_cpx *fout) {
    int i;
    celt_assert2(fin != fout, "In-place FFT not supported");
    /* Bit-reverse the input */
    for (i = 0; i < st->nfft; i++) fout[st->bitrev[i]] = fin[i];
    for (i = 0; i < st->nfft; i++) fout[i].i = -fout[i].i;
    opus_fft_impl(st, fout);
    for (i = 0; i < st->nfft; i++) fout[i].i = -fout[i].i;
}
