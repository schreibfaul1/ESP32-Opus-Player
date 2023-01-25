/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE Ogg CONTAINER SOURCE CODE.              *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE OggVorbis SOURCE CODE IS (C) COPYRIGHT 1994-2018             *
 * by the Xiph.Org Foundation http://www.xiph.org/                  *
 *                                                                  *
 ********************************************************************

 function: code raw packets into framed OggSquish stream and
           decode Ogg streams back into raw packets

 note: The CRC code is directly derived from public domain code by
 Ross Williams (ross@guest.adelaide.edu.au).  See docs/framing.html
 for details.

 ********************************************************************/

#include "config.h"
#include <pgmspace.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "ogg.h"

static const uint32_t crc_lookup[8][256] PROGMEM = {
    {0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005, 0x2608edb8,
     0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
     0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75, 0x6a1936c8, 0x6ed82b7f, 0x639b0da6,
     0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
     0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84,
     0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d, 0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
     0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a,
     0xec7dd02d, 0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
     0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca, 0x7897ab07,
     0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
     0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba, 0xaca5c697, 0xa864db20, 0xa527fdf9,
     0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
     0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b,
     0xf771768c, 0xfa325055, 0xfef34de2, 0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
     0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c,
     0x774bb0eb, 0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
     0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b, 0x0315d626,
     0x07d4cb91, 0x0a97ed48, 0x0e56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
     0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b, 0xd727bbb6, 0xd3e6a601, 0xdea580d8,
     0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
     0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a,
     0x8cf30bad, 0x81b02d74, 0x857130c3, 0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
     0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093,
     0x65c52d24, 0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
     0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654, 0xc5a92679,
     0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
     0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4, 0x89b8fd09, 0x8d79e0be, 0x803ac667,
     0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
     0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4},

    {0x00000000, 0xd219c1dc, 0xa0f29e0f, 0x72eb5fd3, 0x452421a9, 0x973de075, 0xe5d6bfa6, 0x37cf7e7a, 0x8a484352,
     0x5851828e, 0x2abadd5d, 0xf8a31c81, 0xcf6c62fb, 0x1d75a327, 0x6f9efcf4, 0xbd873d28, 0x10519b13, 0xc2485acf,
     0xb0a3051c, 0x62bac4c0, 0x5575baba, 0x876c7b66, 0xf58724b5, 0x279ee569, 0x9a19d841, 0x4800199d, 0x3aeb464e,
     0xe8f28792, 0xdf3df9e8, 0x0d243834, 0x7fcf67e7, 0xadd6a63b, 0x20a33626, 0xf2baf7fa, 0x8051a829, 0x524869f5,
     0x6587178f, 0xb79ed653, 0xc5758980, 0x176c485c, 0xaaeb7574, 0x78f2b4a8, 0x0a19eb7b, 0xd8002aa7, 0xefcf54dd,
     0x3dd69501, 0x4f3dcad2, 0x9d240b0e, 0x30f2ad35, 0xe2eb6ce9, 0x9000333a, 0x4219f2e6, 0x75d68c9c, 0xa7cf4d40,
     0xd5241293, 0x073dd34f, 0xbabaee67, 0x68a32fbb, 0x1a487068, 0xc851b1b4, 0xff9ecfce, 0x2d870e12, 0x5f6c51c1,
     0x8d75901d, 0x41466c4c, 0x935fad90, 0xe1b4f243, 0x33ad339f, 0x04624de5, 0xd67b8c39, 0xa490d3ea, 0x76891236,
     0xcb0e2f1e, 0x1917eec2, 0x6bfcb111, 0xb9e570cd, 0x8e2a0eb7, 0x5c33cf6b, 0x2ed890b8, 0xfcc15164, 0x5117f75f,
     0x830e3683, 0xf1e56950, 0x23fca88c, 0x1433d6f6, 0xc62a172a, 0xb4c148f9, 0x66d88925, 0xdb5fb40d, 0x094675d1,
     0x7bad2a02, 0xa9b4ebde, 0x9e7b95a4, 0x4c625478, 0x3e890bab, 0xec90ca77, 0x61e55a6a, 0xb3fc9bb6, 0xc117c465,
     0x130e05b9, 0x24c17bc3, 0xf6d8ba1f, 0x8433e5cc, 0x562a2410, 0xebad1938, 0x39b4d8e4, 0x4b5f8737, 0x994646eb,
     0xae893891, 0x7c90f94d, 0x0e7ba69e, 0xdc626742, 0x71b4c179, 0xa3ad00a5, 0xd1465f76, 0x035f9eaa, 0x3490e0d0,
     0xe689210c, 0x94627edf, 0x467bbf03, 0xfbfc822b, 0x29e543f7, 0x5b0e1c24, 0x8917ddf8, 0xbed8a382, 0x6cc1625e,
     0x1e2a3d8d, 0xcc33fc51, 0x828cd898, 0x50951944, 0x227e4697, 0xf067874b, 0xc7a8f931, 0x15b138ed, 0x675a673e,
     0xb543a6e2, 0x08c49bca, 0xdadd5a16, 0xa83605c5, 0x7a2fc419, 0x4de0ba63, 0x9ff97bbf, 0xed12246c, 0x3f0be5b0,
     0x92dd438b, 0x40c48257, 0x322fdd84, 0xe0361c58, 0xd7f96222, 0x05e0a3fe, 0x770bfc2d, 0xa5123df1, 0x189500d9,
     0xca8cc105, 0xb8679ed6, 0x6a7e5f0a, 0x5db12170, 0x8fa8e0ac, 0xfd43bf7f, 0x2f5a7ea3, 0xa22feebe, 0x70362f62,
     0x02dd70b1, 0xd0c4b16d, 0xe70bcf17, 0x35120ecb, 0x47f95118, 0x95e090c4, 0x2867adec, 0xfa7e6c30, 0x889533e3,
     0x5a8cf23f, 0x6d438c45, 0xbf5a4d99, 0xcdb1124a, 0x1fa8d396, 0xb27e75ad, 0x6067b471, 0x128ceba2, 0xc0952a7e,
     0xf75a5404, 0x254395d8, 0x57a8ca0b, 0x85b10bd7, 0x383636ff, 0xea2ff723, 0x98c4a8f0, 0x4add692c, 0x7d121756,
     0xaf0bd68a, 0xdde08959, 0x0ff94885, 0xc3cab4d4, 0x11d37508, 0x63382adb, 0xb121eb07, 0x86ee957d, 0x54f754a1,
     0x261c0b72, 0xf405caae, 0x4982f786, 0x9b9b365a, 0xe9706989, 0x3b69a855, 0x0ca6d62f, 0xdebf17f3, 0xac544820,
     0x7e4d89fc, 0xd39b2fc7, 0x0182ee1b, 0x7369b1c8, 0xa1707014, 0x96bf0e6e, 0x44a6cfb2, 0x364d9061, 0xe45451bd,
     0x59d36c95, 0x8bcaad49, 0xf921f29a, 0x2b383346, 0x1cf74d3c, 0xceee8ce0, 0xbc05d333, 0x6e1c12ef, 0xe36982f2,
     0x3170432e, 0x439b1cfd, 0x9182dd21, 0xa64da35b, 0x74546287, 0x06bf3d54, 0xd4a6fc88, 0x6921c1a0, 0xbb38007c,
     0xc9d35faf, 0x1bca9e73, 0x2c05e009, 0xfe1c21d5, 0x8cf77e06, 0x5eeebfda, 0xf33819e1, 0x2121d83d, 0x53ca87ee,
     0x81d34632, 0xb61c3848, 0x6405f994, 0x16eea647, 0xc4f7679b, 0x79705ab3, 0xab699b6f, 0xd982c4bc, 0x0b9b0560,
     0x3c547b1a, 0xee4dbac6, 0x9ca6e515, 0x4ebf24c9},

    {0x00000000, 0x01d8ac87, 0x03b1590e, 0x0269f589, 0x0762b21c, 0x06ba1e9b, 0x04d3eb12, 0x050b4795, 0x0ec56438,
     0x0f1dc8bf, 0x0d743d36, 0x0cac91b1, 0x09a7d624, 0x087f7aa3, 0x0a168f2a, 0x0bce23ad, 0x1d8ac870, 0x1c5264f7,
     0x1e3b917e, 0x1fe33df9, 0x1ae87a6c, 0x1b30d6eb, 0x19592362, 0x18818fe5, 0x134fac48, 0x129700cf, 0x10fef546,
     0x112659c1, 0x142d1e54, 0x15f5b2d3, 0x179c475a, 0x1644ebdd, 0x3b1590e0, 0x3acd3c67, 0x38a4c9ee, 0x397c6569,
     0x3c7722fc, 0x3daf8e7b, 0x3fc67bf2, 0x3e1ed775, 0x35d0f4d8, 0x3408585f, 0x3661add6, 0x37b90151, 0x32b246c4,
     0x336aea43, 0x31031fca, 0x30dbb34d, 0x269f5890, 0x2747f417, 0x252e019e, 0x24f6ad19, 0x21fdea8c, 0x2025460b,
     0x224cb382, 0x23941f05, 0x285a3ca8, 0x2982902f, 0x2beb65a6, 0x2a33c921, 0x2f388eb4, 0x2ee02233, 0x2c89d7ba,
     0x2d517b3d, 0x762b21c0, 0x77f38d47, 0x759a78ce, 0x7442d449, 0x714993dc, 0x70913f5b, 0x72f8cad2, 0x73206655,
     0x78ee45f8, 0x7936e97f, 0x7b5f1cf6, 0x7a87b071, 0x7f8cf7e4, 0x7e545b63, 0x7c3daeea, 0x7de5026d, 0x6ba1e9b0,
     0x6a794537, 0x6810b0be, 0x69c81c39, 0x6cc35bac, 0x6d1bf72b, 0x6f7202a2, 0x6eaaae25, 0x65648d88, 0x64bc210f,
     0x66d5d486, 0x670d7801, 0x62063f94, 0x63de9313, 0x61b7669a, 0x606fca1d, 0x4d3eb120, 0x4ce61da7, 0x4e8fe82e,
     0x4f5744a9, 0x4a5c033c, 0x4b84afbb, 0x49ed5a32, 0x4835f6b5, 0x43fbd518, 0x4223799f, 0x404a8c16, 0x41922091,
     0x44996704, 0x4541cb83, 0x47283e0a, 0x46f0928d, 0x50b47950, 0x516cd5d7, 0x5305205e, 0x52dd8cd9, 0x57d6cb4c,
     0x560e67cb, 0x54679242, 0x55bf3ec5, 0x5e711d68, 0x5fa9b1ef, 0x5dc04466, 0x5c18e8e1, 0x5913af74, 0x58cb03f3,
     0x5aa2f67a, 0x5b7a5afd, 0xec564380, 0xed8eef07, 0xefe71a8e, 0xee3fb609, 0xeb34f19c, 0xeaec5d1b, 0xe885a892,
     0xe95d0415, 0xe29327b8, 0xe34b8b3f, 0xe1227eb6, 0xe0fad231, 0xe5f195a4, 0xe4293923, 0xe640ccaa, 0xe798602d,
     0xf1dc8bf0, 0xf0042777, 0xf26dd2fe, 0xf3b57e79, 0xf6be39ec, 0xf766956b, 0xf50f60e2, 0xf4d7cc65, 0xff19efc8,
     0xfec1434f, 0xfca8b6c6, 0xfd701a41, 0xf87b5dd4, 0xf9a3f153, 0xfbca04da, 0xfa12a85d, 0xd743d360, 0xd69b7fe7,
     0xd4f28a6e, 0xd52a26e9, 0xd021617c, 0xd1f9cdfb, 0xd3903872, 0xd24894f5, 0xd986b758, 0xd85e1bdf, 0xda37ee56,
     0xdbef42d1, 0xdee40544, 0xdf3ca9c3, 0xdd555c4a, 0xdc8df0cd, 0xcac91b10, 0xcb11b797, 0xc978421e, 0xc8a0ee99,
     0xcdaba90c, 0xcc73058b, 0xce1af002, 0xcfc25c85, 0xc40c7f28, 0xc5d4d3af, 0xc7bd2626, 0xc6658aa1, 0xc36ecd34,
     0xc2b661b3, 0xc0df943a, 0xc10738bd, 0x9a7d6240, 0x9ba5cec7, 0x99cc3b4e, 0x981497c9, 0x9d1fd05c, 0x9cc77cdb,
     0x9eae8952, 0x9f7625d5, 0x94b80678, 0x9560aaff, 0x97095f76, 0x96d1f3f1, 0x93dab464, 0x920218e3, 0x906bed6a,
     0x91b341ed, 0x87f7aa30, 0x862f06b7, 0x8446f33e, 0x859e5fb9, 0x8095182c, 0x814db4ab, 0x83244122, 0x82fceda5,
     0x8932ce08, 0x88ea628f, 0x8a839706, 0x8b5b3b81, 0x8e507c14, 0x8f88d093, 0x8de1251a, 0x8c39899d, 0xa168f2a0,
     0xa0b05e27, 0xa2d9abae, 0xa3010729, 0xa60a40bc, 0xa7d2ec3b, 0xa5bb19b2, 0xa463b535, 0xafad9698, 0xae753a1f,
     0xac1ccf96, 0xadc46311, 0xa8cf2484, 0xa9178803, 0xab7e7d8a, 0xaaa6d10d, 0xbce23ad0, 0xbd3a9657, 0xbf5363de,
     0xbe8bcf59, 0xbb8088cc, 0xba58244b, 0xb831d1c2, 0xb9e97d45, 0xb2275ee8, 0xb3fff26f, 0xb19607e6, 0xb04eab61,
     0xb545ecf4, 0xb49d4073, 0xb6f4b5fa, 0xb72c197d},

    {0x00000000, 0xdc6d9ab7, 0xbc1a28d9, 0x6077b26e, 0x7cf54c05, 0xa098d6b2, 0xc0ef64dc, 0x1c82fe6b, 0xf9ea980a,
     0x258702bd, 0x45f0b0d3, 0x999d2a64, 0x851fd40f, 0x59724eb8, 0x3905fcd6, 0xe5686661, 0xf7142da3, 0x2b79b714,
     0x4b0e057a, 0x97639fcd, 0x8be161a6, 0x578cfb11, 0x37fb497f, 0xeb96d3c8, 0x0efeb5a9, 0xd2932f1e, 0xb2e49d70,
     0x6e8907c7, 0x720bf9ac, 0xae66631b, 0xce11d175, 0x127c4bc2, 0xeae946f1, 0x3684dc46, 0x56f36e28, 0x8a9ef49f,
     0x961c0af4, 0x4a719043, 0x2a06222d, 0xf66bb89a, 0x1303defb, 0xcf6e444c, 0xaf19f622, 0x73746c95, 0x6ff692fe,
     0xb39b0849, 0xd3ecba27, 0x0f812090, 0x1dfd6b52, 0xc190f1e5, 0xa1e7438b, 0x7d8ad93c, 0x61082757, 0xbd65bde0,
     0xdd120f8e, 0x017f9539, 0xe417f358, 0x387a69ef, 0x580ddb81, 0x84604136, 0x98e2bf5d, 0x448f25ea, 0x24f89784,
     0xf8950d33, 0xd1139055, 0x0d7e0ae2, 0x6d09b88c, 0xb164223b, 0xade6dc50, 0x718b46e7, 0x11fcf489, 0xcd916e3e,
     0x28f9085f, 0xf49492e8, 0x94e32086, 0x488eba31, 0x540c445a, 0x8861deed, 0xe8166c83, 0x347bf634, 0x2607bdf6,
     0xfa6a2741, 0x9a1d952f, 0x46700f98, 0x5af2f1f3, 0x869f6b44, 0xe6e8d92a, 0x3a85439d, 0xdfed25fc, 0x0380bf4b,
     0x63f70d25, 0xbf9a9792, 0xa31869f9, 0x7f75f34e, 0x1f024120, 0xc36fdb97, 0x3bfad6a4, 0xe7974c13, 0x87e0fe7d,
     0x5b8d64ca, 0x470f9aa1, 0x9b620016, 0xfb15b278, 0x277828cf, 0xc2104eae, 0x1e7dd419, 0x7e0a6677, 0xa267fcc0,
     0xbee502ab, 0x6288981c, 0x02ff2a72, 0xde92b0c5, 0xcceefb07, 0x108361b0, 0x70f4d3de, 0xac994969, 0xb01bb702,
     0x6c762db5, 0x0c019fdb, 0xd06c056c, 0x3504630d, 0xe969f9ba, 0x891e4bd4, 0x5573d163, 0x49f12f08, 0x959cb5bf,
     0xf5eb07d1, 0x29869d66, 0xa6e63d1d, 0x7a8ba7aa, 0x1afc15c4, 0xc6918f73, 0xda137118, 0x067eebaf, 0x660959c1,
     0xba64c376, 0x5f0ca517, 0x83613fa0, 0xe3168dce, 0x3f7b1779, 0x23f9e912, 0xff9473a5, 0x9fe3c1cb, 0x438e5b7c,
     0x51f210be, 0x8d9f8a09, 0xede83867, 0x3185a2d0, 0x2d075cbb, 0xf16ac60c, 0x911d7462, 0x4d70eed5, 0xa81888b4,
     0x74751203, 0x1402a06d, 0xc86f3ada, 0xd4edc4b1, 0x08805e06, 0x68f7ec68, 0xb49a76df, 0x4c0f7bec, 0x9062e15b,
     0xf0155335, 0x2c78c982, 0x30fa37e9, 0xec97ad5e, 0x8ce01f30, 0x508d8587, 0xb5e5e3e6, 0x69887951, 0x09ffcb3f,
     0xd5925188, 0xc910afe3, 0x157d3554, 0x750a873a, 0xa9671d8d, 0xbb1b564f, 0x6776ccf8, 0x07017e96, 0xdb6ce421,
     0xc7ee1a4a, 0x1b8380fd, 0x7bf43293, 0xa799a824, 0x42f1ce45, 0x9e9c54f2, 0xfeebe69c, 0x22867c2b, 0x3e048240,
     0xe26918f7, 0x821eaa99, 0x5e73302e, 0x77f5ad48, 0xab9837ff, 0xcbef8591, 0x17821f26, 0x0b00e14d, 0xd76d7bfa,
     0xb71ac994, 0x6b775323, 0x8e1f3542, 0x5272aff5, 0x32051d9b, 0xee68872c, 0xf2ea7947, 0x2e87e3f0, 0x4ef0519e,
     0x929dcb29, 0x80e180eb, 0x5c8c1a5c, 0x3cfba832, 0xe0963285, 0xfc14ccee, 0x20795659, 0x400ee437, 0x9c637e80,
     0x790b18e1, 0xa5668256, 0xc5113038, 0x197caa8f, 0x05fe54e4, 0xd993ce53, 0xb9e47c3d, 0x6589e68a, 0x9d1cebb9,
     0x4171710e, 0x2106c360, 0xfd6b59d7, 0xe1e9a7bc, 0x3d843d0b, 0x5df38f65, 0x819e15d2, 0x64f673b3, 0xb89be904,
     0xd8ec5b6a, 0x0481c1dd, 0x18033fb6, 0xc46ea501, 0xa419176f, 0x78748dd8, 0x6a08c61a, 0xb6655cad, 0xd612eec3,
     0x0a7f7474, 0x16fd8a1f, 0xca9010a8, 0xaae7a2c6, 0x768a3871, 0x93e25e10, 0x4f8fc4a7, 0x2ff876c9, 0xf395ec7e,
     0xef171215, 0x337a88a2, 0x530d3acc, 0x8f60a07b},

    {0x00000000, 0x490d678d, 0x921acf1a, 0xdb17a897, 0x20f48383, 0x69f9e40e, 0xb2ee4c99, 0xfbe32b14, 0x41e90706,
     0x08e4608b, 0xd3f3c81c, 0x9afeaf91, 0x611d8485, 0x2810e308, 0xf3074b9f, 0xba0a2c12, 0x83d20e0c, 0xcadf6981,
     0x11c8c116, 0x58c5a69b, 0xa3268d8f, 0xea2bea02, 0x313c4295, 0x78312518, 0xc23b090a, 0x8b366e87, 0x5021c610,
     0x192ca19d, 0xe2cf8a89, 0xabc2ed04, 0x70d54593, 0x39d8221e, 0x036501af, 0x4a686622, 0x917fceb5, 0xd872a938,
     0x2391822c, 0x6a9ce5a1, 0xb18b4d36, 0xf8862abb, 0x428c06a9, 0x0b816124, 0xd096c9b3, 0x999bae3e, 0x6278852a,
     0x2b75e2a7, 0xf0624a30, 0xb96f2dbd, 0x80b70fa3, 0xc9ba682e, 0x12adc0b9, 0x5ba0a734, 0xa0438c20, 0xe94eebad,
     0x3259433a, 0x7b5424b7, 0xc15e08a5, 0x88536f28, 0x5344c7bf, 0x1a49a032, 0xe1aa8b26, 0xa8a7ecab, 0x73b0443c,
     0x3abd23b1, 0x06ca035e, 0x4fc764d3, 0x94d0cc44, 0xddddabc9, 0x263e80dd, 0x6f33e750, 0xb4244fc7, 0xfd29284a,
     0x47230458, 0x0e2e63d5, 0xd539cb42, 0x9c34accf, 0x67d787db, 0x2edae056, 0xf5cd48c1, 0xbcc02f4c, 0x85180d52,
     0xcc156adf, 0x1702c248, 0x5e0fa5c5, 0xa5ec8ed1, 0xece1e95c, 0x37f641cb, 0x7efb2646, 0xc4f10a54, 0x8dfc6dd9,
     0x56ebc54e, 0x1fe6a2c3, 0xe40589d7, 0xad08ee5a, 0x761f46cd, 0x3f122140, 0x05af02f1, 0x4ca2657c, 0x97b5cdeb,
     0xdeb8aa66, 0x255b8172, 0x6c56e6ff, 0xb7414e68, 0xfe4c29e5, 0x444605f7, 0x0d4b627a, 0xd65ccaed, 0x9f51ad60,
     0x64b28674, 0x2dbfe1f9, 0xf6a8496e, 0xbfa52ee3, 0x867d0cfd, 0xcf706b70, 0x1467c3e7, 0x5d6aa46a, 0xa6898f7e,
     0xef84e8f3, 0x34934064, 0x7d9e27e9, 0xc7940bfb, 0x8e996c76, 0x558ec4e1, 0x1c83a36c, 0xe7608878, 0xae6deff5,
     0x757a4762, 0x3c7720ef, 0x0d9406bc, 0x44996131, 0x9f8ec9a6, 0xd683ae2b, 0x2d60853f, 0x646de2b2, 0xbf7a4a25,
     0xf6772da8, 0x4c7d01ba, 0x05706637, 0xde67cea0, 0x976aa92d, 0x6c898239, 0x2584e5b4, 0xfe934d23, 0xb79e2aae,
     0x8e4608b0, 0xc74b6f3d, 0x1c5cc7aa, 0x5551a027, 0xaeb28b33, 0xe7bfecbe, 0x3ca84429, 0x75a523a4, 0xcfaf0fb6,
     0x86a2683b, 0x5db5c0ac, 0x14b8a721, 0xef5b8c35, 0xa656ebb8, 0x7d41432f, 0x344c24a2, 0x0ef10713, 0x47fc609e,
     0x9cebc809, 0xd5e6af84, 0x2e058490, 0x6708e31d, 0xbc1f4b8a, 0xf5122c07, 0x4f180015, 0x06156798, 0xdd02cf0f,
     0x940fa882, 0x6fec8396, 0x26e1e41b, 0xfdf64c8c, 0xb4fb2b01, 0x8d23091f, 0xc42e6e92, 0x1f39c605, 0x5634a188,
     0xadd78a9c, 0xe4daed11, 0x3fcd4586, 0x76c0220b, 0xccca0e19, 0x85c76994, 0x5ed0c103, 0x17dda68e, 0xec3e8d9a,
     0xa533ea17, 0x7e244280, 0x3729250d, 0x0b5e05e2, 0x4253626f, 0x9944caf8, 0xd049ad75, 0x2baa8661, 0x62a7e1ec,
     0xb9b0497b, 0xf0bd2ef6, 0x4ab702e4, 0x03ba6569, 0xd8adcdfe, 0x91a0aa73, 0x6a438167, 0x234ee6ea, 0xf8594e7d,
     0xb15429f0, 0x888c0bee, 0xc1816c63, 0x1a96c4f4, 0x539ba379, 0xa878886d, 0xe175efe0, 0x3a624777, 0x736f20fa,
     0xc9650ce8, 0x80686b65, 0x5b7fc3f2, 0x1272a47f, 0xe9918f6b, 0xa09ce8e6, 0x7b8b4071, 0x328627fc, 0x083b044d,
     0x413663c0, 0x9a21cb57, 0xd32cacda, 0x28cf87ce, 0x61c2e043, 0xbad548d4, 0xf3d82f59, 0x49d2034b, 0x00df64c6,
     0xdbc8cc51, 0x92c5abdc, 0x692680c8, 0x202be745, 0xfb3c4fd2, 0xb231285f, 0x8be90a41, 0xc2e46dcc, 0x19f3c55b,
     0x50fea2d6, 0xab1d89c2, 0xe210ee4f, 0x390746d8, 0x700a2155, 0xca000d47, 0x830d6aca, 0x581ac25d, 0x1117a5d0,
     0xeaf48ec4, 0xa3f9e949, 0x78ee41de, 0x31e32653},

    {0x00000000, 0x1b280d78, 0x36501af0, 0x2d781788, 0x6ca035e0, 0x77883898, 0x5af02f10, 0x41d82268, 0xd9406bc0,
     0xc26866b8, 0xef107130, 0xf4387c48, 0xb5e05e20, 0xaec85358, 0x83b044d0, 0x989849a8, 0xb641ca37, 0xad69c74f,
     0x8011d0c7, 0x9b39ddbf, 0xdae1ffd7, 0xc1c9f2af, 0xecb1e527, 0xf799e85f, 0x6f01a1f7, 0x7429ac8f, 0x5951bb07,
     0x4279b67f, 0x03a19417, 0x1889996f, 0x35f18ee7, 0x2ed9839f, 0x684289d9, 0x736a84a1, 0x5e129329, 0x453a9e51,
     0x04e2bc39, 0x1fcab141, 0x32b2a6c9, 0x299aabb1, 0xb102e219, 0xaa2aef61, 0x8752f8e9, 0x9c7af591, 0xdda2d7f9,
     0xc68ada81, 0xebf2cd09, 0xf0dac071, 0xde0343ee, 0xc52b4e96, 0xe853591e, 0xf37b5466, 0xb2a3760e, 0xa98b7b76,
     0x84f36cfe, 0x9fdb6186, 0x0743282e, 0x1c6b2556, 0x311332de, 0x2a3b3fa6, 0x6be31dce, 0x70cb10b6, 0x5db3073e,
     0x469b0a46, 0xd08513b2, 0xcbad1eca, 0xe6d50942, 0xfdfd043a, 0xbc252652, 0xa70d2b2a, 0x8a753ca2, 0x915d31da,
     0x09c57872, 0x12ed750a, 0x3f956282, 0x24bd6ffa, 0x65654d92, 0x7e4d40ea, 0x53355762, 0x481d5a1a, 0x66c4d985,
     0x7decd4fd, 0x5094c375, 0x4bbcce0d, 0x0a64ec65, 0x114ce11d, 0x3c34f695, 0x271cfbed, 0xbf84b245, 0xa4acbf3d,
     0x89d4a8b5, 0x92fca5cd, 0xd32487a5, 0xc80c8add, 0xe5749d55, 0xfe5c902d, 0xb8c79a6b, 0xa3ef9713, 0x8e97809b,
     0x95bf8de3, 0xd467af8b, 0xcf4fa2f3, 0xe237b57b, 0xf91fb803, 0x6187f1ab, 0x7aaffcd3, 0x57d7eb5b, 0x4cffe623,
     0x0d27c44b, 0x160fc933, 0x3b77debb, 0x205fd3c3, 0x0e86505c, 0x15ae5d24, 0x38d64aac, 0x23fe47d4, 0x622665bc,
     0x790e68c4, 0x54767f4c, 0x4f5e7234, 0xd7c63b9c, 0xccee36e4, 0xe196216c, 0xfabe2c14, 0xbb660e7c, 0xa04e0304,
     0x8d36148c, 0x961e19f4, 0xa5cb3ad3, 0xbee337ab, 0x939b2023, 0x88b32d5b, 0xc96b0f33, 0xd243024b, 0xff3b15c3,
     0xe41318bb, 0x7c8b5113, 0x67a35c6b, 0x4adb4be3, 0x51f3469b, 0x102b64f3, 0x0b03698b, 0x267b7e03, 0x3d53737b,
     0x138af0e4, 0x08a2fd9c, 0x25daea14, 0x3ef2e76c, 0x7f2ac504, 0x6402c87c, 0x497adff4, 0x5252d28c, 0xcaca9b24,
     0xd1e2965c, 0xfc9a81d4, 0xe7b28cac, 0xa66aaec4, 0xbd42a3bc, 0x903ab434, 0x8b12b94c, 0xcd89b30a, 0xd6a1be72,
     0xfbd9a9fa, 0xe0f1a482, 0xa12986ea, 0xba018b92, 0x97799c1a, 0x8c519162, 0x14c9d8ca, 0x0fe1d5b2, 0x2299c23a,
     0x39b1cf42, 0x7869ed2a, 0x6341e052, 0x4e39f7da, 0x5511faa2, 0x7bc8793d, 0x60e07445, 0x4d9863cd, 0x56b06eb5,
     0x17684cdd, 0x0c4041a5, 0x2138562d, 0x3a105b55, 0xa28812fd, 0xb9a01f85, 0x94d8080d, 0x8ff00575, 0xce28271d,
     0xd5002a65, 0xf8783ded, 0xe3503095, 0x754e2961, 0x6e662419, 0x431e3391, 0x58363ee9, 0x19ee1c81, 0x02c611f9,
     0x2fbe0671, 0x34960b09, 0xac0e42a1, 0xb7264fd9, 0x9a5e5851, 0x81765529, 0xc0ae7741, 0xdb867a39, 0xf6fe6db1,
     0xedd660c9, 0xc30fe356, 0xd827ee2e, 0xf55ff9a6, 0xee77f4de, 0xafafd6b6, 0xb487dbce, 0x99ffcc46, 0x82d7c13e,
     0x1a4f8896, 0x016785ee, 0x2c1f9266, 0x37379f1e, 0x76efbd76, 0x6dc7b00e, 0x40bfa786, 0x5b97aafe, 0x1d0ca0b8,
     0x0624adc0, 0x2b5cba48, 0x3074b730, 0x71ac9558, 0x6a849820, 0x47fc8fa8, 0x5cd482d0, 0xc44ccb78, 0xdf64c600,
     0xf21cd188, 0xe934dcf0, 0xa8ecfe98, 0xb3c4f3e0, 0x9ebce468, 0x8594e910, 0xab4d6a8f, 0xb06567f7, 0x9d1d707f,
     0x86357d07, 0xc7ed5f6f, 0xdcc55217, 0xf1bd459f, 0xea9548e7, 0x720d014f, 0x69250c37, 0x445d1bbf, 0x5f7516c7,
     0x1ead34af, 0x058539d7, 0x28fd2e5f, 0x33d52327},

    {0x00000000, 0x4f576811, 0x9eaed022, 0xd1f9b833, 0x399cbdf3, 0x76cbd5e2, 0xa7326dd1, 0xe86505c0, 0x73397be6,
     0x3c6e13f7, 0xed97abc4, 0xa2c0c3d5, 0x4aa5c615, 0x05f2ae04, 0xd40b1637, 0x9b5c7e26, 0xe672f7cc, 0xa9259fdd,
     0x78dc27ee, 0x378b4fff, 0xdfee4a3f, 0x90b9222e, 0x41409a1d, 0x0e17f20c, 0x954b8c2a, 0xda1ce43b, 0x0be55c08,
     0x44b23419, 0xacd731d9, 0xe38059c8, 0x3279e1fb, 0x7d2e89ea, 0xc824f22f, 0x87739a3e, 0x568a220d, 0x19dd4a1c,
     0xf1b84fdc, 0xbeef27cd, 0x6f169ffe, 0x2041f7ef, 0xbb1d89c9, 0xf44ae1d8, 0x25b359eb, 0x6ae431fa, 0x8281343a,
     0xcdd65c2b, 0x1c2fe418, 0x53788c09, 0x2e5605e3, 0x61016df2, 0xb0f8d5c1, 0xffafbdd0, 0x17cab810, 0x589dd001,
     0x89646832, 0xc6330023, 0x5d6f7e05, 0x12381614, 0xc3c1ae27, 0x8c96c636, 0x64f3c3f6, 0x2ba4abe7, 0xfa5d13d4,
     0xb50a7bc5, 0x9488f9e9, 0xdbdf91f8, 0x0a2629cb, 0x457141da, 0xad14441a, 0xe2432c0b, 0x33ba9438, 0x7cedfc29,
     0xe7b1820f, 0xa8e6ea1e, 0x791f522d, 0x36483a3c, 0xde2d3ffc, 0x917a57ed, 0x4083efde, 0x0fd487cf, 0x72fa0e25,
     0x3dad6634, 0xec54de07, 0xa303b616, 0x4b66b3d6, 0x0431dbc7, 0xd5c863f4, 0x9a9f0be5, 0x01c375c3, 0x4e941dd2,
     0x9f6da5e1, 0xd03acdf0, 0x385fc830, 0x7708a021, 0xa6f11812, 0xe9a67003, 0x5cac0bc6, 0x13fb63d7, 0xc202dbe4,
     0x8d55b3f5, 0x6530b635, 0x2a67de24, 0xfb9e6617, 0xb4c90e06, 0x2f957020, 0x60c21831, 0xb13ba002, 0xfe6cc813,
     0x1609cdd3, 0x595ea5c2, 0x88a71df1, 0xc7f075e0, 0xbadefc0a, 0xf589941b, 0x24702c28, 0x6b274439, 0x834241f9,
     0xcc1529e8, 0x1dec91db, 0x52bbf9ca, 0xc9e787ec, 0x86b0effd, 0x574957ce, 0x181e3fdf, 0xf07b3a1f, 0xbf2c520e,
     0x6ed5ea3d, 0x2182822c, 0x2dd0ee65, 0x62878674, 0xb37e3e47, 0xfc295656, 0x144c5396, 0x5b1b3b87, 0x8ae283b4,
     0xc5b5eba5, 0x5ee99583, 0x11befd92, 0xc04745a1, 0x8f102db0, 0x67752870, 0x28224061, 0xf9dbf852, 0xb68c9043,
     0xcba219a9, 0x84f571b8, 0x550cc98b, 0x1a5ba19a, 0xf23ea45a, 0xbd69cc4b, 0x6c907478, 0x23c71c69, 0xb89b624f,
     0xf7cc0a5e, 0x2635b26d, 0x6962da7c, 0x8107dfbc, 0xce50b7ad, 0x1fa90f9e, 0x50fe678f, 0xe5f41c4a, 0xaaa3745b,
     0x7b5acc68, 0x340da479, 0xdc68a1b9, 0x933fc9a8, 0x42c6719b, 0x0d91198a, 0x96cd67ac, 0xd99a0fbd, 0x0863b78e,
     0x4734df9f, 0xaf51da5f, 0xe006b24e, 0x31ff0a7d, 0x7ea8626c, 0x0386eb86, 0x4cd18397, 0x9d283ba4, 0xd27f53b5,
     0x3a1a5675, 0x754d3e64, 0xa4b48657, 0xebe3ee46, 0x70bf9060, 0x3fe8f871, 0xee114042, 0xa1462853, 0x49232d93,
     0x06744582, 0xd78dfdb1, 0x98da95a0, 0xb958178c, 0xf60f7f9d, 0x27f6c7ae, 0x68a1afbf, 0x80c4aa7f, 0xcf93c26e,
     0x1e6a7a5d, 0x513d124c, 0xca616c6a, 0x8536047b, 0x54cfbc48, 0x1b98d459, 0xf3fdd199, 0xbcaab988, 0x6d5301bb,
     0x220469aa, 0x5f2ae040, 0x107d8851, 0xc1843062, 0x8ed35873, 0x66b65db3, 0x29e135a2, 0xf8188d91, 0xb74fe580,
     0x2c139ba6, 0x6344f3b7, 0xb2bd4b84, 0xfdea2395, 0x158f2655, 0x5ad84e44, 0x8b21f677, 0xc4769e66, 0x717ce5a3,
     0x3e2b8db2, 0xefd23581, 0xa0855d90, 0x48e05850, 0x07b73041, 0xd64e8872, 0x9919e063, 0x02459e45, 0x4d12f654,
     0x9ceb4e67, 0xd3bc2676, 0x3bd923b6, 0x748e4ba7, 0xa577f394, 0xea209b85, 0x970e126f, 0xd8597a7e, 0x09a0c24d,
     0x46f7aa5c, 0xae92af9c, 0xe1c5c78d, 0x303c7fbe, 0x7f6b17af, 0xe4376989, 0xab600198, 0x7a99b9ab, 0x35ced1ba,
     0xddabd47a, 0x92fcbc6b, 0x43050458, 0x0c526c49},

    {0x00000000, 0x5ba1dcca, 0xb743b994, 0xece2655e, 0x6a466e9f, 0x31e7b255, 0xdd05d70b, 0x86a40bc1, 0xd48cdd3e,
     0x8f2d01f4, 0x63cf64aa, 0x386eb860, 0xbecab3a1, 0xe56b6f6b, 0x09890a35, 0x5228d6ff, 0xadd8a7cb, 0xf6797b01,
     0x1a9b1e5f, 0x413ac295, 0xc79ec954, 0x9c3f159e, 0x70dd70c0, 0x2b7cac0a, 0x79547af5, 0x22f5a63f, 0xce17c361,
     0x95b61fab, 0x1312146a, 0x48b3c8a0, 0xa451adfe, 0xfff07134, 0x5f705221, 0x04d18eeb, 0xe833ebb5, 0xb392377f,
     0x35363cbe, 0x6e97e074, 0x8275852a, 0xd9d459e0, 0x8bfc8f1f, 0xd05d53d5, 0x3cbf368b, 0x671eea41, 0xe1bae180,
     0xba1b3d4a, 0x56f95814, 0x0d5884de, 0xf2a8f5ea, 0xa9092920, 0x45eb4c7e, 0x1e4a90b4, 0x98ee9b75, 0xc34f47bf,
     0x2fad22e1, 0x740cfe2b, 0x262428d4, 0x7d85f41e, 0x91679140, 0xcac64d8a, 0x4c62464b, 0x17c39a81, 0xfb21ffdf,
     0xa0802315, 0xbee0a442, 0xe5417888, 0x09a31dd6, 0x5202c11c, 0xd4a6cadd, 0x8f071617, 0x63e57349, 0x3844af83,
     0x6a6c797c, 0x31cda5b6, 0xdd2fc0e8, 0x868e1c22, 0x002a17e3, 0x5b8bcb29, 0xb769ae77, 0xecc872bd, 0x13380389,
     0x4899df43, 0xa47bba1d, 0xffda66d7, 0x797e6d16, 0x22dfb1dc, 0xce3dd482, 0x959c0848, 0xc7b4deb7, 0x9c15027d,
     0x70f76723, 0x2b56bbe9, 0xadf2b028, 0xf6536ce2, 0x1ab109bc, 0x4110d576, 0xe190f663, 0xba312aa9, 0x56d34ff7,
     0x0d72933d, 0x8bd698fc, 0xd0774436, 0x3c952168, 0x6734fda2, 0x351c2b5d, 0x6ebdf797, 0x825f92c9, 0xd9fe4e03,
     0x5f5a45c2, 0x04fb9908, 0xe819fc56, 0xb3b8209c, 0x4c4851a8, 0x17e98d62, 0xfb0be83c, 0xa0aa34f6, 0x260e3f37,
     0x7dafe3fd, 0x914d86a3, 0xcaec5a69, 0x98c48c96, 0xc365505c, 0x2f873502, 0x7426e9c8, 0xf282e209, 0xa9233ec3,
     0x45c15b9d, 0x1e608757, 0x79005533, 0x22a189f9, 0xce43eca7, 0x95e2306d, 0x13463bac, 0x48e7e766, 0xa4058238,
     0xffa45ef2, 0xad8c880d, 0xf62d54c7, 0x1acf3199, 0x416eed53, 0xc7cae692, 0x9c6b3a58, 0x70895f06, 0x2b2883cc,
     0xd4d8f2f8, 0x8f792e32, 0x639b4b6c, 0x383a97a6, 0xbe9e9c67, 0xe53f40ad, 0x09dd25f3, 0x527cf939, 0x00542fc6,
     0x5bf5f30c, 0xb7179652, 0xecb64a98, 0x6a124159, 0x31b39d93, 0xdd51f8cd, 0x86f02407, 0x26700712, 0x7dd1dbd8,
     0x9133be86, 0xca92624c, 0x4c36698d, 0x1797b547, 0xfb75d019, 0xa0d40cd3, 0xf2fcda2c, 0xa95d06e6, 0x45bf63b8,
     0x1e1ebf72, 0x98bab4b3, 0xc31b6879, 0x2ff90d27, 0x7458d1ed, 0x8ba8a0d9, 0xd0097c13, 0x3ceb194d, 0x674ac587,
     0xe1eece46, 0xba4f128c, 0x56ad77d2, 0x0d0cab18, 0x5f247de7, 0x0485a12d, 0xe867c473, 0xb3c618b9, 0x35621378,
     0x6ec3cfb2, 0x8221aaec, 0xd9807626, 0xc7e0f171, 0x9c412dbb, 0x70a348e5, 0x2b02942f, 0xada69fee, 0xf6074324,
     0x1ae5267a, 0x4144fab0, 0x136c2c4f, 0x48cdf085, 0xa42f95db, 0xff8e4911, 0x792a42d0, 0x228b9e1a, 0xce69fb44,
     0x95c8278e, 0x6a3856ba, 0x31998a70, 0xdd7bef2e, 0x86da33e4, 0x007e3825, 0x5bdfe4ef, 0xb73d81b1, 0xec9c5d7b,
     0xbeb48b84, 0xe515574e, 0x09f73210, 0x5256eeda, 0xd4f2e51b, 0x8f5339d1, 0x63b15c8f, 0x38108045, 0x9890a350,
     0xc3317f9a, 0x2fd31ac4, 0x7472c60e, 0xf2d6cdcf, 0xa9771105, 0x4595745b, 0x1e34a891, 0x4c1c7e6e, 0x17bda2a4,
     0xfb5fc7fa, 0xa0fe1b30, 0x265a10f1, 0x7dfbcc3b, 0x9119a965, 0xcab875af, 0x3548049b, 0x6ee9d851, 0x820bbd0f,
     0xd9aa61c5, 0x5f0e6a04, 0x04afb6ce, 0xe84dd390, 0xb3ec0f5a, 0xe1c4d9a5, 0xba65056f, 0x56876031, 0x0d26bcfb,
     0x8b82b73a, 0xd0236bf0, 0x3cc10eae, 0x6760d264}};

/* A complete description of Ogg framing exists in docs/framing.html */
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_page_version(const ogg_page *og){
  return((int32_t)(og->header[4]));
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_page_continued(const ogg_page *og){
  return((int32_t)(og->header[5]&0x01));
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_page_bos(const ogg_page *og){
  return((int32_t)(og->header[5]&0x02));
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_page_eos(const ogg_page *og){
  return((int32_t)(og->header[5]&0x04));
}
//----------------------------------------------------------------------------------------------------------------------
int64_t ogg_page_granulepos(const ogg_page *og){
  uint8_t *page=og->header;
  uint64_t granulepos=page[13]&(0xff);
  granulepos= (granulepos<<8)|(page[12]&0xff);
  granulepos= (granulepos<<8)|(page[11]&0xff);
  granulepos= (granulepos<<8)|(page[10]&0xff);
  granulepos= (granulepos<<8)|(page[9]&0xff);
  granulepos= (granulepos<<8)|(page[8]&0xff);
  granulepos= (granulepos<<8)|(page[7]&0xff);
  granulepos= (granulepos<<8)|(page[6]&0xff);
  return((int64_t)granulepos);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_page_pageno(const ogg_page *og) {
    return ((int32_t)((uint32_t)og->header[18]) |
                  ((uint32_t)og->header[19] << 8) |
                  ((uint32_t)og->header[20] << 16) |
                  ((uint32_t)og->header[21] << 24));
}
//----------------------------------------------------------------------------------------------------------------------
/* returns the number of packets that are completed on this page (if the leading packet is begun on a previous page,
   but ends on this page, it's counted */
/* NOTE:
   If a page consists of a packet begun on a previous page, and a new packet begun (but not completed) on this page,
   the return will be:
     ogg_page_packets(page)   ==1,
     ogg_page_continued(page) !=0

   If a page happens to be a single packet that was begun on a previous page, and spans to the next page (in the case
   of a three or more page packet), the return will be:
     ogg_page_packets(page)   ==0,
     ogg_page_continued(page) !=0
*/

int32_t ogg_page_packets(const ogg_page *og) {
    int32_t i, n = og->header[26], count = 0;
    for(i = 0; i < n; i++)
        if(og->header[27 + i] < 255) count++;
    return (count);
}
//----------------------------------------------------------------------------------------------------------------------
/* init the encode/decode logical stream state */
int32_t ogg_stream_init(ogg_stream_state *os, int32_t serialno) {
    if(os) {
        memset(os, 0, sizeof(*os));
        os->body_storage = 16 * 1024;
        os->lacing_storage = 1024;

        os->body_data = malloc(os->body_storage * sizeof(*os->body_data));
        os->lacing_vals = malloc(os->lacing_storage * sizeof(*os->lacing_vals));
        os->granule_vals = malloc(os->lacing_storage * sizeof(*os->granule_vals));

        if(!os->body_data || !os->lacing_vals || !os->granule_vals) {
            ogg_stream_clear(os);
            return -1;
        }

        os->serialno = serialno;

        return (0);
    }
    return (-1);
}
//----------------------------------------------------------------------------------------------------------------------
/* async/delayed error detection for the ogg_stream_state */
int32_t ogg_stream_check(ogg_stream_state *os) {
    if(!os || !os->body_data) return -1;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/* _clear does not free os, only the non-flat storage within */
int32_t ogg_stream_clear(ogg_stream_state *os) {
    if(os) {
        if(os->body_data) free(os->body_data);
        if(os->lacing_vals) free(os->lacing_vals);
        if(os->granule_vals) free(os->granule_vals);

        memset(os, 0, sizeof(*os));
    }
    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_stream_destroy(ogg_stream_state *os){
  if(os){
    ogg_stream_clear(os);
    free(os);
  }
  return(0);
}
//----------------------------------------------------------------------------------------------------------------------
/* Helpers for ogg_stream_encode; this keeps the structure and what's happening fairly clear */
static int32_t _os_body_expand(ogg_stream_state *os, int32_t needed) {
    if(os->body_storage - needed <= os->body_fill) {
        int32_t  body_storage;
        void *ret;
        if(os->body_storage > LONG_MAX - needed) {
            ogg_stream_clear(os);
            return -1;
        }
        body_storage = os->body_storage + needed;
        if(body_storage < LONG_MAX - 1024) body_storage += 1024;
        ret = realloc(os->body_data, body_storage * sizeof(*os->body_data));
        if(!ret) {
            ogg_stream_clear(os);
            return -1;
        }
        os->body_storage = body_storage;
        os->body_data = ret;
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
static int32_t _os_lacing_expand(ogg_stream_state *os,int32_t needed){
  if(os->lacing_storage-needed<=os->lacing_fill){
    int32_t lacing_storage;
    void *ret;
    if(os->lacing_storage>LONG_MAX-needed){
      ogg_stream_clear(os);
      return -1;
    }
    lacing_storage=os->lacing_storage+needed;
    if(lacing_storage<LONG_MAX-32)lacing_storage+=32;
    ret=realloc(os->lacing_vals,lacing_storage*sizeof(*os->lacing_vals));
    if(!ret){
      ogg_stream_clear(os);
      return -1;
    }
    os->lacing_vals=ret;
    ret=realloc(os->granule_vals,lacing_storage*
                     sizeof(*os->granule_vals));
    if(!ret){
      ogg_stream_clear(os);
      return -1;
    }
    os->granule_vals=ret;
    os->lacing_storage=lacing_storage;
  }
  return 0;
}
//----------------------------------------------------------------------------------------------------------------------
/* checksum the page */
/* Direct table CRC; note that this will be faster in the future if we perform the checksum simultaneously with other copies */

static uint32_t _os_update_crc(uint32_t crc, uint8_t *buffer, int32_t size) {
    while(size >= 8) {
        crc ^= ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) |
               ((uint32_t)buffer[3]);

        crc = crc_lookup[7][crc >> 24] ^ crc_lookup[6][(crc >> 16) & 0xFF] ^ crc_lookup[5][(crc >> 8) & 0xFF] ^
              crc_lookup[4][crc & 0xFF] ^ crc_lookup[3][buffer[4]] ^ crc_lookup[2][buffer[5]] ^
              crc_lookup[1][buffer[6]] ^ crc_lookup[0][buffer[7]];

        buffer += 8;
        size -= 8;
    }

    while(size--) crc = (crc << 8) ^ crc_lookup[0][((crc >> 24) & 0xff) ^ *buffer++];
    return crc;
}
//----------------------------------------------------------------------------------------------------------------------
void ogg_page_checksum_set(ogg_page *og) {
    if(og) {
        uint32_t crc_reg = 0;

        /* safety; needed for API behavior, but not framing code */
        og->header[22] = 0;
        og->header[23] = 0;
        og->header[24] = 0;
        og->header[25] = 0;

        crc_reg = _os_update_crc(crc_reg, og->header, og->header_len);
        crc_reg = _os_update_crc(crc_reg, og->body, og->body_len);

        og->header[22] = (uint8_t)(crc_reg & 0xff);
        og->header[23] = (uint8_t)((crc_reg >> 8) & 0xff);
        og->header[24] = (uint8_t)((crc_reg >> 16) & 0xff);
        og->header[25] = (uint8_t)((crc_reg >> 24) & 0xff);
    }
}
//----------------------------------------------------------------------------------------------------------------------
/* submit data to the internal buffer of the framing engine */
int32_t ogg_stream_iovecin(ogg_stream_state *os, ogg_iovec_t *iov, int32_t count, int32_t e_o_s, int64_t granulepos) {
    int32_t bytes = 0, lacing_vals;
    int32_t  i;

    if(ogg_stream_check(os)) return -1;
    if(!iov) return 0;

    for(i = 0; i < count; ++i) {
        if(iov[i].iov_len > LONG_MAX) return -1;
        if(bytes > LONG_MAX - (int32_t)iov[i].iov_len) return -1;
        bytes += (int32_t)iov[i].iov_len;
    }
    lacing_vals = bytes / 255 + 1;

    if(os->body_returned) {
        /* advance packet data according to the body_returned pointer. We
           had to keep it around to return a pointer into the buffer last
           call */

        os->body_fill -= os->body_returned;
        if(os->body_fill) memmove(os->body_data, os->body_data + os->body_returned, os->body_fill);
        os->body_returned = 0;
    }

    /* make sure we have the buffer storage */
    if(_os_body_expand(os, bytes) || _os_lacing_expand(os, lacing_vals)) return -1;

    /* Copy in the submitted packet.  Yes, the copy is a waste; this is
       the liability of overly clean abstraction for the time being.  It
       will actually be fairly easy to eliminate the extra copy in the
       future */

    for(i = 0; i < count; ++i) {
        memcpy(os->body_data + os->body_fill, iov[i].iov_base, iov[i].iov_len);
        os->body_fill += (int32_t)iov[i].iov_len;
    }

    /* Store lacing vals for this packet */
    for(i = 0; i < lacing_vals - 1; i++) {
        os->lacing_vals[os->lacing_fill + i] = 255;
        os->granule_vals[os->lacing_fill + i] = os->granulepos;
    }
    os->lacing_vals[os->lacing_fill + i] = bytes % 255;
    os->granulepos = os->granule_vals[os->lacing_fill + i] = granulepos;

    /* flag the first segment as the beginning of the packet */
    os->lacing_vals[os->lacing_fill] |= 0x100;

    os->lacing_fill += lacing_vals;

    /* for the sake of completeness */
    os->packetno++;

    if(e_o_s) os->e_o_s = 1;

    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_stream_packetin(ogg_stream_state *os, ogg_packet *op) {
    ogg_iovec_t iov;
    iov.iov_base = op->packet;
    iov.iov_len = op->bytes;
    return ogg_stream_iovecin(os, &iov, 1, op->e_o_s, op->granulepos);
}
//----------------------------------------------------------------------------------------------------------------------
/* Conditionally flush a page; force==0 will only flush nominal-size pages,
   force==1 forces us to flush a page regardless of page size so int32_t as there's any data available at all. */
static int32_t ogg_stream_flush_i(ogg_stream_state *os, ogg_page *og, int32_t force, int32_t nfill) {
    int32_t     i;
    int32_t     vals = 0;
    int32_t     maxvals = (os->lacing_fill > 255 ? 255 : os->lacing_fill);
    int32_t     bytes = 0;
    int32_t    acc = 0;
    int64_t granule_pos = -1;

    if(ogg_stream_check(os)) return (0);
    if(maxvals == 0) return (0);

    /* construct a page */
    /* decide how many segments to include */

    /* If this is the initial header case, the first page must only include the initial header packet */
    if(os->b_o_s == 0) { /* 'initial header page' case */
        granule_pos = 0;
        for(vals = 0; vals < maxvals; vals++) {
            if((os->lacing_vals[vals] & 0x0ff) < 255) {
                vals++;
                break;
            }
        }
    }
    else {
        /* The extra packets_done, packet_just_done logic here attempts to do two things:
           1) Don't unnecessarily span pages.
           2) Unless necessary, don't flush pages if there are less than four packets on them; this expands page size
              to reduce unnecessary overhead if incoming packets are large.
           These are not necessary behaviors, just 'always better than naive flushing' without requiring an application
           to explicitly request a specific optimized behavior. We'll want an explicit behavior setup pathway
           eventually as well. */

        int32_t packets_done = 0;
        int32_t packet_just_done = 0;
        for(vals = 0; vals < maxvals; vals++) {
            if(acc > nfill && packet_just_done >= 4) {
                force = 1;
                break;
            }
            acc += os->lacing_vals[vals] & 0x0ff;
            if((os->lacing_vals[vals] & 0xff) < 255) {
                granule_pos = os->granule_vals[vals];
                packet_just_done = ++packets_done;
            }
            else
                packet_just_done = 0;
        }
        if(vals == 255) force = 1;
    }

    if(!force) return (0);

    /* construct the header in temp storage */
    memcpy(os->header, "OggS", 4);

    /* stream structure version */
    os->header[4] = 0x00;

    /* continued packet flag? */
    os->header[5] = 0x00;
    if((os->lacing_vals[0] & 0x100) == 0) os->header[5] |= 0x01;
    /* first page flag? */
    if(os->b_o_s == 0) os->header[5] |= 0x02;
    /* last page flag? */
    if(os->e_o_s && os->lacing_fill == vals) os->header[5] |= 0x04;
    os->b_o_s = 1;

    /* 64 bits of PCM position */
    for(i = 6; i < 14; i++) {
        os->header[i] = (uint8_t)(granule_pos & 0xff);
        granule_pos >>= 8;
    }

    /* 32 bits of stream serial number */
    {
        int32_t serialno = os->serialno;
        for(i = 14; i < 18; i++) {
            os->header[i] = (uint8_t)(serialno & 0xff);
            serialno >>= 8;
        }
    }

    /* 32 bits of page counter (we have both counter and page header
       because this val can roll over) */
    if(os->pageno == -1)
        os->pageno = 0; /* because someone called
                           stream_reset; this would be a
                           strange thing to do in an
                           encode stream, but it has
                           plausible uses */
    {
        int32_t pageno = os->pageno++;
        for(i = 18; i < 22; i++) {
            os->header[i] = (uint8_t)(pageno & 0xff);
            pageno >>= 8;
        }
    }

    /* zero for computation; filled in later */
    os->header[22] = 0;
    os->header[23] = 0;
    os->header[24] = 0;
    os->header[25] = 0;

    /* segment table */
    os->header[26] = (uint8_t)(vals & 0xff);
    for(i = 0; i < vals; i++) bytes += os->header[i + 27] = (uint8_t)(os->lacing_vals[i] & 0xff);

    /* set pointers in the ogg_page struct */
    og->header = os->header;
    og->header_len = os->header_fill = vals + 27;
    og->body = os->body_data + os->body_returned;
    og->body_len = bytes;

    /* advance the lacing data and set the body_returned pointer */

    os->lacing_fill -= vals;
    memmove(os->lacing_vals, os->lacing_vals + vals, os->lacing_fill * sizeof(*os->lacing_vals));
    memmove(os->granule_vals, os->granule_vals + vals, os->lacing_fill * sizeof(*os->granule_vals));
    os->body_returned += bytes;

    /* calculate the checksum */

    ogg_page_checksum_set(og);

    /* done */
    return (1);
}
//----------------------------------------------------------------------------------------------------------------------
/* This will flush remaining packets into a page (returning nonzero), even if there is not enough data to trigger a
   flush normally (undersized page). If there are no packets or partial packets to flush, ogg_stream_flush returns 0.
    Note that ogg_stream_flush will try to flush a normal sized page like ogg_stream_pageout; a call to ogg_stream_flush
    does not guarantee that all packets have flushed. Only a return value of 0 from ogg_stream_flush indicates all
    packet data is flushed into pages.

    Since ogg_stream_flush will flush the last page in a stream even if it's undersized, you almost certainly want to
    use ogg_stream_pageout (and *not* ogg_stream_flush) unless you specifically need to flush a page regardless of size
    in the middle of a stream. */

int32_t ogg_stream_flush(ogg_stream_state *os, ogg_page *og) {
    return ogg_stream_flush_i(os, og, 1, 4096);
}
//----------------------------------------------------------------------------------------------------------------------
/* Like the above, but an argument is provided to adjust the nominal page size for applications which are smart enough
   to provide their own delay based flushing */

int32_t ogg_stream_flush_fill(ogg_stream_state *os, ogg_page *og, int32_t nfill) {
    return ogg_stream_flush_i(os, og, 1, nfill);
}
//----------------------------------------------------------------------------------------------------------------------
/* This constructs pages from buffered packet segments. The pointers returned are to static buffers; do not free. The
   returned buffers are good only until the next call (using the same ogg_stream_state) */

int32_t ogg_stream_pageout(ogg_stream_state *os, ogg_page *og) {
    int32_t force = 0;
    if(ogg_stream_check(os)) return 0;

    if((os->e_o_s && os->lacing_fill) || /* 'were done, now flush' case */
       (os->lacing_fill && !os->b_o_s))  /* 'initial header page' case */
        force = 1;

    return (ogg_stream_flush_i(os, og, force, 4096));
}
//----------------------------------------------------------------------------------------------------------------------
/* Like the above, but an argument is provided to adjust the nominal page size for applications which are smart enough
   to provide their own delay based flushing */

int32_t ogg_stream_pageout_fill(ogg_stream_state *os, ogg_page *og, int32_t nfill) {
    int32_t force = 0;
    if(ogg_stream_check(os)) return 0;

    if((os->e_o_s && os->lacing_fill) || /* 'were done, now flush' case */
       (os->lacing_fill && !os->b_o_s))  /* 'initial header page' case */
        force = 1;

    return (ogg_stream_flush_i(os, og, force, nfill));
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_stream_eos(ogg_stream_state *os) {
    if(ogg_stream_check(os)) return 1;
    return os->e_o_s;
}
//----------------------------------------------------------------------------------------------------------------------
/* This has two layers to place more of the multi-serialno and paging control in the application's hands.  First, we
   expose a data buffer using ogg_sync_buffer(). The app either copies into the buffer, or passes it directly to read(),
   etc. We then call ogg_sync_wrote() to tell how many bytes we just added.
   Pages are returned (pointers into the buffer in ogg_sync_state) by ogg_sync_pageout(). The page is then submitted to
   ogg_stream_pagein() along with the appropriate ogg_stream_state* (ie, matching serialno). We then get raw packets
   out calling ogg_stream_packetout() with a ogg_stream_state. */

/* initialize the struct to a known state */
int32_t ogg_sync_init(ogg_sync_state *oy) {
    if(oy) {
        oy->storage = -1; /* used as a readiness flag */
        memset(oy, 0, sizeof(*oy));
    }
    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
/* clear non-flat storage within */
int32_t ogg_sync_clear(ogg_sync_state *oy) {
    if(oy) {
        if(oy->data) free(oy->data);
        memset(oy, 0, sizeof(*oy));
    }
    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_sync_destroy(ogg_sync_state *oy) {
    if(oy) {
        ogg_sync_clear(oy);
        free(oy);
    }
    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_sync_check(ogg_sync_state *oy) {
    if(oy->storage < 0) return -1;
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
char *ogg_sync_buffer(ogg_sync_state *oy, int32_t size) {
    if(ogg_sync_check(oy)) return NULL;

    /* first, clear out any space that has been previously returned */
    if(oy->returned) {
        oy->fill -= oy->returned;
        if(oy->fill > 0) memmove(oy->data, oy->data + oy->returned, oy->fill);
        oy->returned = 0;
    }

    if(size > oy->storage - oy->fill) {
        /* We need to extend the internal buffer */
        int32_t newsize = size + oy->fill + 4096; /* an extra page to be nice */
        void   *ret;

        if(oy->data) ret = realloc(oy->data, newsize);
        else
            ret = malloc(newsize);
        if(!ret) {
            ogg_sync_clear(oy);
            return NULL;
        }
        oy->data = ret;
        oy->storage = newsize;
    }

    /* expose a segment at least as large as requested at the fill mark */
    return ((char *)oy->data + oy->fill);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_sync_wrote(ogg_sync_state *oy, int32_t bytes) {
    if(ogg_sync_check(oy)) return -1;
    if(oy->fill + bytes > oy->storage) return -1;
    oy->fill += bytes;
    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
/* sync the stream.  This is meant to be useful for finding page boundaries.
   return values for this:
   -n) skipped n bytes
    0) page not ready; more data (no bytes skipped)
    n) page synced at current location; page length n bytes */

int32_t ogg_sync_pageseek(ogg_sync_state *oy, ogg_page *og) {
    uint8_t *page = oy->data + oy->returned;
    uint8_t *next;
    int32_t  bytes = oy->fill - oy->returned;

    if(ogg_sync_check(oy)) return 0;

    if(oy->headerbytes == 0) {
        int32_t headerbytes, i;
        if(bytes < 27) return (0); /* not enough for a header */

        /* verify capture pattern */
        if(memcmp(page, "OggS", 4)) goto sync_fail;

        headerbytes = page[26] + 27;
        if(bytes < headerbytes) return (0); /* not enough for header + seg table */

        /* count up body length in the segment table */

        for(i = 0; i < page[26]; i++) oy->bodybytes += page[27 + i];
        oy->headerbytes = headerbytes;
    }

    if(oy->bodybytes + oy->headerbytes > bytes) return (0);

    /* The whole test page is buffered.  Verify the checksum */
    {
        /* Grab the checksum bytes, set the header field to zero */
        char     chksum[4];
        ogg_page log;

        memcpy(chksum, page + 22, 4);
        memset(page + 22, 0, 4);

        /* set up a temp page struct and recompute the checksum */
        log.header = page;
        log.header_len = oy->headerbytes;
        log.body = page + oy->headerbytes;
        log.body_len = oy->bodybytes;
        ogg_page_checksum_set(&log);

        /* Compare */
        if(memcmp(chksum, page + 22, 4)) {
            /* D'oh.  Mismatch! Corrupt page (or miscapture and not a page
               at all) */
            /* replace the computed checksum with the one actually read in */
            memcpy(page + 22, chksum, 4);

            /* Bad checksum. Lose sync */
            goto sync_fail;
        }
    }

    /* yes, have a whole page all ready to go */
    {
        if(og) {
            og->header = page;
            og->header_len = oy->headerbytes;
            og->body = page + oy->headerbytes;
            og->body_len = oy->bodybytes;
        }

        oy->unsynced = 0;
        oy->returned += (bytes = oy->headerbytes + oy->bodybytes);
        oy->headerbytes = 0;
        oy->bodybytes = 0;
        return (bytes);
    }

sync_fail:

    oy->headerbytes = 0;
    oy->bodybytes = 0;

    /* search for possible capture */
    next = memchr(page + 1, 'O', bytes - 1);
    if(!next) next = oy->data + oy->fill;

    oy->returned = (int32_t)(next - oy->data);
    return ((int32_t) - (next - page));
}
//----------------------------------------------------------------------------------------------------------------------
/* add the incoming page to the stream state; we decompose the page into packet segments here as well. */

int32_t ogg_stream_pagein(ogg_stream_state *os, ogg_page *og) {
    uint8_t *header = og->header;
    uint8_t *body = og->body;
    int32_t  bodysize = og->body_len;
    int32_t  segptr = 0;

    int32_t version = ogg_page_version(og);
    int32_t continued = ogg_page_continued(og);
    int32_t bos = ogg_page_bos(og);
    int32_t eos = ogg_page_eos(og);
    int64_t granulepos = ogg_page_granulepos(og);
    int32_t serialno = getSerialNo(og);
    int32_t pageno = ogg_page_pageno(og);
    int32_t segments = header[26];

    if(ogg_stream_check(os)) return -1;

    /* clean up 'returned data' */
    {
        int32_t lr = os->lacing_returned;
        int32_t br = os->body_returned;

        /* body data */
        if(br) {
            os->body_fill -= br;
            if(os->body_fill) memmove(os->body_data, os->body_data + br, os->body_fill);
            os->body_returned = 0;
        }

        if(lr) {
            /* segment table */
            if(os->lacing_fill - lr) {
                memmove(os->lacing_vals, os->lacing_vals + lr, (os->lacing_fill - lr) * sizeof(*os->lacing_vals));
                memmove(os->granule_vals, os->granule_vals + lr, (os->lacing_fill - lr) * sizeof(*os->granule_vals));
            }
            os->lacing_fill -= lr;
            os->lacing_packet -= lr;
            os->lacing_returned = 0;
        }
    }

    /* check the serial number */
    if(serialno != os->serialno) return (-1);
    if(version > 0) return (-1);

    if(_os_lacing_expand(os, segments + 1)) return -1;

    /* are we in sequence? */
    if(pageno != os->pageno) {
        int32_t i;

        /* unroll previous partial packet (if any) */
        for(i = os->lacing_packet; i < os->lacing_fill; i++) os->body_fill -= os->lacing_vals[i] & 0xff;
        os->lacing_fill = os->lacing_packet;

        /* make a note of dropped data in segment table */
        if(os->pageno != -1) {
            os->lacing_vals[os->lacing_fill++] = 0x400;
            os->lacing_packet++;
        }
    }

    /* are we a 'continued packet' page?  If so, we may need to skip
       some segments */
    if(continued) {
        if(os->lacing_fill < 1 || (os->lacing_vals[os->lacing_fill - 1] & 0xff) < 255 ||
           os->lacing_vals[os->lacing_fill - 1] == 0x400) {
            bos = 0;
            for(; segptr < segments; segptr++) {
                int32_t val = header[27 + segptr];
                body += val;
                bodysize -= val;
                if(val < 255) {
                    segptr++;
                    break;
                }
            }
        }
    }

    if(bodysize) {
        if(_os_body_expand(os, bodysize)) return -1;
        memcpy(os->body_data + os->body_fill, body, bodysize);
        os->body_fill += bodysize;
    }

    {
        int32_t saved = -1;
        while(segptr < segments) {
            int32_t val = header[27 + segptr];
            os->lacing_vals[os->lacing_fill] = val;
            os->granule_vals[os->lacing_fill] = -1;

            if(bos) {
                os->lacing_vals[os->lacing_fill] |= 0x100;
                bos = 0;
            }

            if(val < 255) saved = os->lacing_fill;

            os->lacing_fill++;
            segptr++;

            if(val < 255) os->lacing_packet = os->lacing_fill;
        }

        /* set the granulepos on the last granuleval of the last full packet */
        if(saved != -1) { os->granule_vals[saved] = granulepos; }
    }

    if(eos) {
        os->e_o_s = 1;
        if(os->lacing_fill > 0) os->lacing_vals[os->lacing_fill - 1] |= 0x200;
    }

    os->pageno = pageno + 1;

    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
/* clear things to an initial state.  Good to call, eg, before seeking */
int32_t ogg_sync_reset(ogg_sync_state *oy) {
    if(ogg_sync_check(oy)) return -1;

    oy->fill = 0;
    oy->returned = 0;
    oy->unsynced = 0;
    oy->headerbytes = 0;
    oy->bodybytes = 0;
    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_stream_reset(ogg_stream_state *os) {
    if(ogg_stream_check(os)) return -1;

    os->body_fill = 0;
    os->body_returned = 0;

    os->lacing_fill = 0;
    os->lacing_packet = 0;
    os->lacing_returned = 0;

    os->header_fill = 0;

    os->e_o_s = 0;
    os->b_o_s = 0;
    os->pageno = -1;
    os->packetno = 0;
    os->granulepos = 0;

    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_stream_reset_serialno(ogg_stream_state *os, int32_t serialno) {
    if(ogg_stream_check(os)) return -1;
    ogg_stream_reset(os);
    os->serialno = serialno;
    return (0);
}
//----------------------------------------------------------------------------------------------------------------------
static int32_t _packetout(ogg_stream_state *os, ogg_packet *op, int32_t adv) {
    /* The last part of decode. We have the stream broken into packet
       segments.  Now we need to group them into packets (or return the
       out of sync markers) */

    int32_t ptr = os->lacing_returned;

    if(os->lacing_packet <= ptr) return (0);

    if(os->lacing_vals[ptr] & 0x400) {
        /* we need to tell the codec there's a gap; it might need to
           handle previous packet dependencies. */
        os->lacing_returned++;
        os->packetno++;
        return (-1);
    }

    if(!op && !adv)
        return (1); /* just using peek as an inexpensive way
                       to ask if there's a whole packet
                       waiting */

    /* Gather the whole packet. We'll have no holes or a partial packet */
    {
        int32_t size = os->lacing_vals[ptr] & 0xff;
        int32_t bytes = size;
        int32_t eos = os->lacing_vals[ptr] & 0x200; /* last packet of the stream? */
        int32_t bos = os->lacing_vals[ptr] & 0x100; /* first packet of the stream? */

        while(size == 255) {
            int32_t val = os->lacing_vals[++ptr];
            size = val & 0xff;
            if(val & 0x200) eos = 0x200;
            bytes += size;
        }

        if(op) {
            op->e_o_s = eos;
            op->b_o_s = bos;
            op->packet = os->body_data + os->body_returned;
            op->packetno = os->packetno;
            op->granulepos = os->granule_vals[ptr];
            op->bytes = bytes;
        }

        if(adv) {
            os->body_returned += bytes;
            os->lacing_returned = ptr + 1;
            os->packetno++;
        }
    }
    return (1);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_stream_packetout(ogg_stream_state *os, ogg_packet *op) {
    if(ogg_stream_check(os)) return 0;
    return _packetout(os, op, 1);
}
//----------------------------------------------------------------------------------------------------------------------
int32_t ogg_stream_packetpeek(ogg_stream_state *os, ogg_packet *op) {
    if(ogg_stream_check(os)) return 0;
    return _packetout(os, op, 0);
}
//----------------------------------------------------------------------------------------------------------------------
void ogg_packet_clear(ogg_packet *op) {
    free(op->packet);
    memset(op, 0, sizeof(*op));
}
//----------------------------------------------------------------------------------------------------------------------
int32_t getSerialNo(const ogg_page *og) {
    return ((int32_t)((uint32_t)og->header[14]) | ((uint32_t)og->header[15] << 8) | ((uint32_t)og->header[16] << 16) |
            ((uint32_t)og->header[17] << 24));
}