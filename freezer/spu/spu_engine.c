/* SPU engine.
 * Copyright (C) 2010 Francois Galea <fgalea@free.fr>
 *
 * This file is part of Freezer.
 *
 * Freezer is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * Freezer is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Freezer.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <spu_intrinsics.h>
#include <spu_mfcio.h>                 /* constant declarations for the MFC */
#include <stdio.h>
#include <string.h>

#if __GNUC__ < 4
#  include <sys/send_to_ppe.h>
#elif __GNUC_MINOR__ == 1
#  include <sys/send_to_ppe.h>
#else
#  include <sys/syscall.h>
#endif

#include "spu_engine.h"
#include "cos_sin18_v.h"

#define DMA_TAG 0
#define NSCAN 8
#define MAX_WIDTH 1920
#define TR_COUNT 32
#define MAX_TD_COUNT 32
#define DRAW_TD_COUNT 64

#define FNAN (1.0f/0.0f)

#define S_E 0x80, 0x80, 0x80, 0x80
#define S_0 0x00, 0x01, 0x02, 0x03
#define S_1 0x04, 0x05, 0x06, 0x07
#define S_2 0x08, 0x09, 0x0a, 0x0b
#define S_3 0x0c, 0x0d, 0x0e, 0x0f
#define S_4 0x10, 0x11, 0x12, 0x13
#define S_5 0x14, 0x15, 0x16, 0x17
#define S_6 0x18, 0x19, 0x1a, 0x1b
#define S_7 0x1c, 0x1d, 0x1e, 0x1f
#define S_E3 0x80, 0x80, 0x80
#define S_RGB 0x80, 3, 7, 11
#define S_0000 (vec_uchar16) { S_0, S_0, S_0, S_0 }
#define S_1111 (vec_uchar16) { S_1, S_1, S_1, S_1 }
#define S_2222 (vec_uchar16) { S_2, S_2, S_2, S_2 }
#define S_3333 (vec_uchar16) { S_3, S_3, S_3, S_3 }
#define S_1___ (vec_uchar16) { S_1, S_E, S_E, S_E }
#define S_2___ (vec_uchar16) { S_2, S_E, S_E, S_E }
#define S_34__ (vec_uchar16) { S_3, S_4, S_E, S_E }
#define S__112 (vec_uchar16) { S_E, S_1, S_1, S_2 }
#define S__021 (vec_uchar16) { S_E, S_0, S_2, S_1 }
#define S_1201 (vec_uchar16) { S_1, S_2, S_0, S_1 }
#define S_201_ (vec_uchar16) { S_2, S_0, S_1, S_E }

#define S___02 (vec_uchar16) { S_E, S_E, S_0, S_2 }
#define S___13 (vec_uchar16) { S_E, S_E, S_1, S_3 }
#define S_04__ (vec_uchar16) { S_0, S_4, S_E, S_E }
#define S_15__ (vec_uchar16) { S_1, S_5, S_E, S_E }
#define S_26__ (vec_uchar16) { S_2, S_6, S_E, S_E }
#define S_37__ (vec_uchar16) { S_3, S_7, S_E, S_E }
#define S_014_ (vec_uchar16) { S_0, S_1, S_4, S_E }
#define S_015_ (vec_uchar16) { S_0, S_1, S_5, S_E }
#define S_016_ (vec_uchar16) { S_0, S_1, S_6, S_E }
#define S_017_ (vec_uchar16) { S_0, S_1, S_7, S_E }
#define S_0124 (vec_uchar16) { S_0, S_1, S_2, S_4 }
#define S_0127 (vec_uchar16) { S_0, S_1, S_2, S_7 }
#define S_0145 (vec_uchar16) { S_0, S_1, S_4, S_5 }
#define S_0163 (vec_uchar16) { S_0, S_1, S_6, S_3 }
#define S_0415 (vec_uchar16) { S_0, S_4, S_1, S_5 }
#define S_1357 (vec_uchar16) { S_1, S_3, S_5, S_7 }
#define S_2637 (vec_uchar16) { S_2, S_6, S_3, S_7 }

#define S_COLOR1 (vec_uchar16) { 0x80, 0x80, 0x80, 1, 0x80, 0x80, 0x80, 2, \
                                 0x80, 0x80, 0x80, 3, 0x80, 0x80, 0x80, 0x80 }
#define S_COLOR2 (vec_uchar16) { S_RGB, S_4, S_E, S_E }
#define S_COLOR3 (vec_uchar16) { S_4, S_RGB, S_E, S_E }
#define S_COLOR4 (vec_uchar16) { S_4, S_5, S_RGB, S_E }
#define S_COLOR5 (vec_uchar16) { S_4, S_5, S_6, S_RGB }

#define SPU_MIN_INT(a, b) si_to_int((qword)spu_sel((vec_int4)si_from_int(a),\
   (vec_int4)si_from_int(b), spu_cmpgt((vec_int4)si_from_int(a),\
   (vec_int4)si_from_int(b))));
#define SPU_MAX0_INT(a) si_to_int(((qword)si_from_int(a)) \
  & (qword)spu_cmpgt((vec_int4)si_from_int(a),(vec_int4)spu_splats(0)))

typedef struct 
{
  vec_float4 x_rl;  /* xr0, xr1, xl0, xl1 */
  vec_float4 dx_rl; /* dxr0, dxr1, dxl0, dxl1 */
  vec_float4 zl_l;  /* zl0, zl1, ll0, ll1 */
  vec_float4 dzl_l; /* dzl0, dzl1, dll0, dll1 */
  vec_float4 uv_l;  /* ul0, ul1, vl0, vl1 */
  vec_float4 duv_l; /* dul0, dul1, dvl0, dvl1 */
  vec_float4 ymmzl; /* ymin, ymax, zi, li */
  vec_uint4 csuv;   /* color, pixelshader, ui, vi */
} triangle_data;

typedef void (*shader)(vec_uint4 *, vec_float4 *, int, int,
    float, float, float, float, float, float, float, float, vec_uint4);

spe_data sped;
spe_3d_data d3d;
spe_frame_data fd __attribute((aligned(128)));

vec_float4 zoom_factor;
vec_float4 array_coef;

const static vec_uchar16 lut_min[8] =
{
  { S_E, S_E, S_E, S_E },
  { S_0, S_1, S_2, S_2 }, /* 0 */
  { S_2, S_0, S_1, S_1 }, /* 2 */
  { S_0, S_1, S_2, S_2 }, /* 0 */
  { S_1, S_2, S_0, S_0 }, /* 1 */
  { S_1, S_2, S_0, S_0 }, /* 1 */
  { S_2, S_0, S_1, S_1 }, /* 2 */
  { S_E, S_E, S_E, S_E }
};

const static vec_uchar16 lut_max[8] =
{
  { S_E, S_E, S_E, S_E },
  { S_2, S_0, S_1, S_1 }, /* 2 */
  { S_1, S_2, S_0, S_0 }, /* 1 */
  { S_1, S_2, S_0, S_0 }, /* 1 */
  { S_0, S_1, S_2, S_2 }, /* 0 */
  { S_2, S_0, S_1, S_1 }, /* 2 */
  { S_0, S_1, S_2, S_2 }, /* 0 */
  { S_E, S_E, S_E, S_E }
};

const static vec_uchar16 lut_ad[8] =
{
  { S_E, S_E, S_E, S_E }, /* y0 = y1 = y2 */
  { S_1, S_1, S_2, S_2 }, /* y2 > y1 = y0 */
  { S_1, S_0, S_2, S_3 }, /* y0 = y1 > y2 */
  { S_E, S_E, S_E, S_E }, /* error */
  { S_0, S_0, S_2, S_2 }, /* y2 = y1 > y0 */
  { S_0, S_1, S_2, S_2 }, /* y2 > y1 > y0 */
  { S_0, S_0, S_2, S_3 }, /* y1 > y2 > y0 */
  { S_E, S_E, S_E, S_E }, /* error */
};

const static vec_uchar16 minlmaxr[4] =
{
  { S_2, S_1, S_E, S_E }, /* r0 <= r1, l0 <= l1 , on prend l0 et r1 */
  { S_3, S_1, S_E, S_E }, /* r0 <= r1, l0 > l1  , on prend l1 et r1 */
  { S_2, S_0, S_E, S_E }, /* r0 > r1, l0 <= l1    -> l0 et r0 */
  { S_3, S_0, S_E, S_E }, /* r0 > r1, l0 > l1     -> l1 et r0 */
};

static __inline__ void wait_for_completion(int tag) {
  mfc_write_tag_mask(1<<tag);
  spu_mfcstat(MFC_TAG_UPDATE_ALL);
}

static __inline__ void spe_barrier(void) {
  static volatile vec_uint4 signal;
  int i;
  void *ls = ((char*)&signal)+12;
  uint32_t expected = (1<<sped.spe_count)-1;
  uint32_t received = 1<<sped.spe_rank;
  signal = spu_promote(received, 3);
  for (i=0; i<sped.spe_count; i++)
    if (i != sped.spe_rank) {
      mfc_sndsig(ls, sped.sig1[i], 4, 0, 0);
    }
  mfc_write_tag_mask(1<<4);
  spu_mfcstat(MFC_TAG_UPDATE_ALL);
  
  while (received != expected) {
    received |= spu_read_signal1();
  }
}


static vec_float4 normal(vec_float4 a, vec_float4 b, vec_float4 c) {
  vec_float4 v1 = b-a;
  vec_float4 v2 = c-b;
  vec_float4 vn = spu_shuffle(v1,v1,S_1201)*spu_shuffle(v2,v2,S_201_)
      - spu_shuffle(v1,v1,S_201_)*spu_shuffle(v2,v2,S_1201);
  vec_float4 vn2 = vn*vn;
  vec_float4 n1 = spu_rsqrte(spu_splats(spu_extract(vn2,0) +
      spu_extract(vn2,1) + spu_extract(vn2,2)));
  return vn * n1;
}  

/* create a rotation and translation matrix (columnwise matrix) */
static void make_rotation(vec_float4 rot[4], vec_float4 r, vec_float4 t) {
  r *= spu_splats(8.0f);
  vec_float4 sin = _cos_sin18_v(r - spu_splats(8.0f));
  vec_float4 cos = _cos_sin18_v(r);
  float sin_a = spu_extract(sin, 0);
  float sin_b = spu_extract(sin, 1);
  float sin_c = spu_extract(sin, 2);
  float cos_a = spu_extract(cos, 0);
  float cos_b = spu_extract(cos, 1);
  float cos_c = spu_extract(cos, 2);

  /* row 0 */
  float r00 = cos_b * cos_c;
  float r10 = -cos_b * sin_c;
  float r20 = sin_b;
  /* row 1 */
  float r01 = sin_a * sin_b * cos_c + cos_a * sin_c;
  float r11 = -sin_a * sin_b * sin_c + cos_a * cos_c;
  float r21 = -sin_a * cos_b;
  /* row 2 */
  float r02 = -cos_a * sin_b * cos_c + sin_a * sin_c;
  float r12 = cos_a * sin_b * sin_c + sin_a * cos_c;
  float r22 = cos_a * cos_b;

  rot[0] = spu_insert(r02, spu_insert(r01, spu_promote(r00, 0), 1), 2);
  rot[1] = spu_insert(r12, spu_insert(r11, spu_promote(r10, 0), 1), 2);
  rot[2] = spu_insert(r22, spu_insert(r21, spu_promote(r20, 0), 1), 2);
  rot[3] = spu_insert(1.0f, t, 3);
}

/* Draw a single-colored z-buffered horizontal line between x0 and x1 */
static void draw_hline_z(vec_uint4 *buf, vec_float4 *zbuf,
    int x0, int x1, float z0, float dz, float l0, float dl,
    float u0, float du, float v0, float dv, vec_uint4 color)
{
  int i = x0 / 4;
  int i_max = (x1+3) / 4 - 1;
  const vec_int4 v_0123 = (vec_int4){ 0, 1, 2, 3 };
  vec_int4 mask = (vec_int4)spu_cmpgt(spu_splats(i*4) + v_0123, x0-1);
  vec_int4 mask2 = (vec_int4)spu_cmpgt(
      spu_splats(x1), spu_splats(i_max*4) + v_0123);
  vec_float4 z = spu_splats(dz)*((vec_float4){0.f, 1.f, 2.f, 3.f})
      + spu_splats(z0 - dz*(x0&3));
  vec_uint4 cmp;
  vec_float4 zi = spu_splats(dz*4);

  vec_float4 zb = zbuf[i];
  cmp = spu_cmpgt(z, zb) & mask;

  mask |= spu_cmpgt(spu_splats(i_max), spu_splats(i));
  while (i < i_max)
  {
    buf[i] = spu_sel(buf[i], color, cmp);
    zbuf[i++] = spu_sel(zb, z, cmp);
    z += zi;
    zb = zbuf[i];
    cmp = spu_cmpgt(z, zb);
  }

  if (i <= i_max)
  {
    mask &= mask2;
    cmp &= mask;
    buf[i] = spu_sel(buf[i], color, cmp);
    zbuf[i] = spu_sel(zb, z, cmp);
  }
}

/* Draw a smooth-shaded z-buffered horizontal line between x0 and x1 */
static void draw_hline_zl(vec_uint4 *buf, vec_float4 *zbuf,
    int x0, int x1, float z0, float dz, float l0, float dl,
    float u0, float du, float v0, float dv, vec_uint4 color)
{
  int i = x0 / 4;
  int i_max = (x1+3) / 4 - 1;
  const vec_int4 v_0123 = (vec_int4){ 0, 1, 2, 3 };
  vec_int4 mask = (vec_int4)spu_cmpgt(spu_splats(i*4) + v_0123, x0-1);
  vec_int4 mask2 = (vec_int4)spu_cmpgt(
      spu_splats(x1), spu_splats(i_max*4) + v_0123);
  vec_float4 z = spu_splats(dz)*((vec_float4){0.f, 1.f, 2.f, 3.f})
      + spu_splats(z0 - dz*(x0&3));
  vec_float4 zi = spu_splats(dz*4);
  vec_float4 l1 = spu_splats(dl)*((vec_float4){0.f, 1.f, 2.f, 3.f})
      + spu_splats(l0 - dl*(x0&3));
  vec_float4 u1 = spu_splats(du)*((vec_float4){0.f, 1.f, 2.f, 3.f})
      + spu_splats(u0 - du*(x0&3));
  vec_float4 v1 = spu_splats(dv)*((vec_float4){0.f, 1.f, 2.f, 3.f})
      + spu_splats(v0 - dv*(x0&3));
  vec_float4 li = spu_splats(dl*4);
  vec_float4 ui = spu_splats(du*4);
  vec_float4 vi = spu_splats(dv*4);
  vec_float4 z1, luv, l, u, v;
  vec_uint4 cmp;
  vec_uint4 colors = {0};

  vec_float4 zb = zbuf[i];
  cmp = spu_cmpgt(z, zb) & mask;

  mask |= spu_cmpgt(spu_splats(i_max), spu_splats(i));
  while (i < i_max)
  {
    z1 = spu_re(z);
    l = l1 * z1;
    u = u1 * z1;
    v = v1 * z1;

    luv = spu_shuffle(spu_shuffle(l, u, S_04__), v, S_014_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR2);
    luv = spu_shuffle(spu_shuffle(l, u, S_15__), v, S_015_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR3);
    luv = spu_shuffle(spu_shuffle(l, u, S_26__), v, S_016_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR4);
    luv = spu_shuffle(spu_shuffle(l, u, S_37__), v, S_017_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR5);

    buf[i] = spu_sel(buf[i], colors, cmp);
    zbuf[i++] = spu_sel(zb, z, cmp);
    z += zi;
    l1 += li;
    u1 += ui;
    v1 += vi;
    zb = zbuf[i];
    cmp = spu_cmpgt(z, zb);
  }

  if (i <= i_max)
  {
    z1 = spu_re(z);
    l = l1 * z1;
    u = u1 * z1;
    v = v1 * z1;
    luv = spu_shuffle(spu_shuffle(l, u, S_04__), v, S_014_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR2);
    luv = spu_shuffle(spu_shuffle(l, u, S_15__), v, S_015_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR3);
    luv = spu_shuffle(spu_shuffle(l, u, S_26__), v, S_016_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR4);
    luv = spu_shuffle(spu_shuffle(l, u, S_37__), v, S_017_);
    colors = spu_shuffle(spu_convtu(luv, 8), colors, S_COLOR5);

    mask &= mask2;
    cmp &= mask;
    buf[i] = spu_sel(buf[i], colors, cmp);
    zbuf[i] = spu_sel(zb, z, cmp);
  }
}

/* draw all visible triangles on a subset of screen scanlines */
static void draw_raster(vec_uint4 *buf, int ymin, uint64_t td_ea, int td_count)
{
  /*const static unsigned int backs[6] = {0x200020, 0x202000, 0x002020, 0x200000,
      0x002000, 0x000020};*/

  const static shader shaders[] = {
    NULL, NULL,
    draw_hline_z,   /* SH_FLAT */
    draw_hline_zl,  /* SH_SMOOTH */
    draw_hline_zl   /* SH_RGBSMOOTH */
  };
  shader shad;
  char zbuf_[MAX_WIDTH*4*NSCAN+127];
  char tds_[DRAW_TD_COUNT*128*2+127];
  vec_float4 *zbuf = (vec_float4*)((((uint32_t)zbuf_)+127)&-128);
  triangle_data *tds = (triangle_data*)((((uint32_t)tds_)+127)&-128);
  triangle_data *td;
  /*vec_uint4 back = spu_splats(backs[sped.spe_rank]);*/
  vec_uint4 back = spu_splats(fd.backgdc);
  int i, j;
  vec_float4 y_max_min = spu_convtf(spu_splats(ymin), 0)
      + (vec_float4) { ((float)NSCAN) - .5f, .5f, 0.f, 0.f };
  vec_float4 y0, y1, ymmzl, y, csuv;
  vec_float4 y_ = spu_convtf(spu_splats(ymin), 0)
      + (vec_float4) { 0.5f, 1.5f, 2.5f, 3.5f };
  vec_float4 xl, xr, dxr0, dxr1, dxl0, dxl1, xr0, xr1, xl0, xl1;
  vec_float4 zl, zll, dzl0, dzl1, zl0, zl1;
  vec_float4 ll, dll0, dll1, ll0, ll1;
  vec_float4 ul, dul0, dul1, ul0, ul1;
  vec_float4 vl, dvl0, dvl1, vl0, vl1;
  vec_float4 xlf, dz, dl, du, dv;
  float dzf, dlf, duf, dvf;
  vec_float4 xrl, dxr, dzr, duv, uvl;
  vec_uint4 cmpl, cmpr;
  vec_int4 xli, xri;
  const vec_float4 h = (vec_float4) { 0.5f, 0.5f, 0.5f, 0.5f };
  vec_float4 z0 = spu_splats(-800.0f);
  vec_int4 zero = spu_splats(0);
  vec_int4 xmax = spu_splats((int)sped.width-1);
  vec_uint4 color;
  vec_uint4 *b;
  vec_float4 *zb;

  int odd = 0;
  uint64_t max_td_ea = td_ea + td_count * 128;
  vec_int4 vcount = (vec_int4)si_from_int(td_count);
  vec_int4 vm = (vec_int4)si_from_int(DRAW_TD_COUNT);
  vec_int4 vn = spu_sel(vcount, vm, spu_cmpgt(vcount, vm));
  vec_int4 vn2;

  vcount -= vn;
  mfc_get(tds, td_ea, si_to_int((qword)vn)*128, odd+2, 0, 0);

  for (i=0; i<MAX_WIDTH*NSCAN/4; i+=8)
  {
    buf[i] = back;
    zbuf[i] = z0;
    buf[i+1] = back;
    zbuf[i+1] = z0;
    buf[i+2] = back;
    zbuf[i+2] = z0;
    buf[i+3] = back;
    zbuf[i+3] = z0;
    buf[i+4] = back;
    zbuf[i+4] = z0;
    buf[i+5] = back;
    zbuf[i+5] = z0;
    buf[i+6] = back;
    zbuf[i+6] = z0;
    buf[i+7] = back;
    zbuf[i+7] = z0;
  }

  odd = 1;
  while (td_ea < max_td_ea) {
    td_ea += si_to_int((qword)vn)*128;
    vn2 = spu_sel(vcount, vm, spu_cmpgt(vcount, vm));
    vcount -= vn2;
    td = tds + odd*DRAW_TD_COUNT;
    mfc_get(td, td_ea, si_to_int((qword)vn2)*128, odd+2, 0, 0);
    odd = 1-odd;
    td = tds + odd*DRAW_TD_COUNT;
    wait_for_completion(odd+2);

    for (i=0; i<si_to_int((qword)vn); i++)
    {
      ymmzl = td[i].ymmzl;
      csuv = (vec_float4)td[i].csuv;
      if ((si_to_uint((qword)spu_gather(spu_cmpgt(y_max_min, ymmzl)))&12) == 8)
      {
        /* yt_min < ymax && yt_max >= ymin */
        shad = shaders[spu_extract((vec_uint4)csuv, 1)];
        y0 = spu_shuffle(ymmzl, ymmzl, S_0000);
        y1 = spu_shuffle(ymmzl, ymmzl, S_1111);
        dxr = td[i].dx_rl;
        xrl = td[i].x_rl;
        dzr = td[i].dzl_l;
        zll = td[i].zl_l;
        duv = td[i].duv_l;
        uvl = td[i].uv_l;
        color = spu_shuffle((vec_uint4)csuv, (vec_uint4)zero, S_0000);
        dz = spu_shuffle(ymmzl, ymmzl, S_2222);
        dl = spu_shuffle(ymmzl, ymmzl, S_3333);
        du = spu_shuffle(csuv, csuv, S_2222);
        dv = spu_shuffle(csuv, csuv, S_3333);
        dzf = spu_extract(dz, 0);
        dlf = spu_extract(dl, 0);
        duf = spu_extract(du, 0);
        dvf = spu_extract(dv, 0);
        dxr0 = spu_shuffle(dxr, dxr, S_0000);
        dxr1 = spu_shuffle(dxr, dxr, S_1111);
        dxl0 = spu_shuffle(dxr, dxr, S_2222);
        dxl1 = spu_shuffle(dxr, dxr, S_3333);
        dzl0 = spu_shuffle(dzr, dzr, S_0000);
        dzl1 = spu_shuffle(dzr, dzr, S_1111);
        dll0 = spu_shuffle(dzr, dzr, S_2222);
        dll1 = spu_shuffle(dzr, dzr, S_3333);
        dul0 = spu_shuffle(duv, duv, S_0000);
        dul1 = spu_shuffle(duv, duv, S_1111);
        dvl0 = spu_shuffle(duv, duv, S_2222);
        dvl1 = spu_shuffle(duv, duv, S_3333);
        y = y_;
        b = buf;
        zb = zbuf;
        for (j=0; j<NSCAN/4; j++) {
          xr0 = spu_shuffle(xrl, xrl, S_0000) + (y-y0) * dxr0;
          xr1 = spu_shuffle(xrl, xrl, S_1111) + (y-y0) * dxr1;
          xl0 = spu_shuffle(xrl, xrl, S_2222) + (y-y0) * dxl0;
          xl1 = spu_shuffle(xrl, xrl, S_3333) + (y-y0) * dxl1;
          zl0 = spu_shuffle(zll, zll, S_0000) + (y-y0) * dzl0;
          zl1 = spu_shuffle(zll, zll, S_1111) + (y-y0) * dzl1;
          ll0 = spu_shuffle(zll, zll, S_2222) + (y-y0) * dll0;
          ll1 = spu_shuffle(zll, zll, S_3333) + (y-y0) * dll1;
          ul0 = spu_shuffle(uvl, uvl, S_0000) + (y-y0) * dul0;
          ul1 = spu_shuffle(uvl, uvl, S_1111) + (y-y0) * dul1;
          vl0 = spu_shuffle(uvl, uvl, S_2222) + (y-y0) * dvl0;
          vl1 = spu_shuffle(uvl, uvl, S_3333) + (y-y0) * dvl1;
          cmpr = spu_cmpgt(xr0, xr1);
          cmpl = spu_cmpgt(xl1, xl0);
          xr = spu_sel(xr0, xr1, cmpr);
          xl = spu_sel(xl0, xl1, cmpl);
          zl = spu_sel(zl0, zl1, cmpl);
          ll = spu_sel(ll0, ll1, cmpl);
          ul = spu_sel(ul0, ul1, cmpl);
          vl = spu_sel(vl0, vl1, cmpl);

          xri = spu_convts(xr+h, 0);
          xli = spu_convts(xl+h, 0);

          xri = spu_sel(xri, xmax, spu_cmpgt(xri, xmax));
          xli = spu_sel(xli, zero, spu_cmpgt(zero, xli));
          xri &= spu_cmpgt(y1, y) & spu_cmpgt(y, y0);

          xlf = spu_convtf(xli, 0)+h-xl;
          zl += xlf * dz;
          ll += xlf * dl;

          shad(b, zb + 0*MAX_WIDTH/4, spu_extract(xli, 0), spu_extract(xri, 0),
              spu_extract(zl, 0), dzf, spu_extract(ll, 0), dlf,
              spu_extract(ul, 0), duf, spu_extract(vl, 0), dvf, color);
          b += sped.pitch/sizeof(vec_uint4);
          shad(b, zb + 1*MAX_WIDTH/4, spu_extract(xli, 1), spu_extract(xri, 1),
              spu_extract(zl, 1), dzf, spu_extract(ll, 1), dlf,
              spu_extract(ul, 1), duf, spu_extract(vl, 1), dvf, color);
          b += sped.pitch/sizeof(vec_uint4);
          shad(b, zb + 2*MAX_WIDTH/4, spu_extract(xli, 2), spu_extract(xri, 2),
              spu_extract(zl, 2), dzf, spu_extract(ll, 2), dlf,
              spu_extract(ul, 2), duf, spu_extract(vl, 2), dvf, color);
          b += sped.pitch/sizeof(vec_uint4);
          shad(b, zb + 3*MAX_WIDTH/4, spu_extract(xli, 3), spu_extract(xri, 3),
              spu_extract(zl, 3), dzf, spu_extract(ll, 3), dlf,
              spu_extract(ul, 3), duf, spu_extract(vl, 3), dvf, color);
          b += sped.pitch/sizeof(vec_uint4);

          y = y + spu_splats(4.0f);
          zb += MAX_WIDTH;
        }
      }
    }
    vn = vn2;
  }
}

static void draw_screen(void)
{
  char buf_[MAX_WIDTH*4*NSCAN*2+127];
  vec_uint4 *buf = (vec_uint4*)((((uint32_t)buf_)+127)&-128);
  vec_int4 td_count[(LIST_COUNT+3)/4];
  uint64_t screen = fd.screen_ea, sc;
  vec_uint4 *ls;
  vec_int4 rem, sz, ymin;
  vec_int4 dma_size = (vec_int4)si_from_int(sped.pitch*NSCAN);
  const vec_int4 dma_max = (vec_int4)si_from_int(16384);
  int i, j;
  int start = sped.spe_rank*NSCAN;
  int inc = sped.spe_count*NSCAN;
  int odd = 0;
  screen += sped.pitch * start;

  /* get ea of 3D effect data */
  mfc_get(buf, d3d.sync_ea, 128, 0, 0, 0);
  wait_for_completion(0);
  for (i=0; i<(LIST_COUNT+3)/4; i++) {
    td_count[i] = (vec_int4)buf[i];
  }
  for (i=start; i<sped.height; i+=inc)
  {
    ymin = (vec_int4)si_from_int(i) & spu_splats(-NSCAN);
    j = si_to_int((qword)spu_convts(
        spu_convtf(ymin,0)*array_coef+spu_splats(.0001f), 0));

    ls = buf+odd*MAX_WIDTH*NSCAN/4;
    draw_raster(ls, i, d3d.td_ea[j], spu_extract(td_count[j/4], j&3));
    /* transfer size > 16 kB */
    rem = dma_size;
    sc = screen;
    while (si_to_int((qword)rem)) {
      sz = spu_sel(rem, dma_max, spu_cmpgt(rem, dma_max));
      mfc_put(ls, sc, si_to_int((qword)sz), odd, 0, 0);
      ls += si_to_int((qword)sz)/sizeof(vec_uint4);
      sc += si_to_int((qword)sz);
      rem -= sz;
    }
    screen += sped.pitch*inc;
    odd = 1 - odd;
    wait_for_completion(odd);
  }
  wait_for_completion(1-odd);
}


struct list_buf {
  vec_int4 td_count[(LIST_COUNT+3)/4];
  triangle_data *td __attribute((aligned(16)));
  int odd __attribute((aligned(16)));
};

static void list_buf_init(struct list_buf *lst, triangle_data *td) {
  int i;
  lst->td = td;
  lst->odd = 0;
  for (i=0; i<(LIST_COUNT+3)/4; i++)
    lst->td_count[i] = spu_splats(0);
}

static void list_buf_flush(struct list_buf *lst) {
  int j, status;
  vec_int4 c[(LIST_COUNT+3)/4];
  static volatile vec_int4 sync[8] __attribute((aligned(128)));
  uint64_t dst_ea;
  uint64_t sync_ea = d3d.sync_ea;
  vec_int4 *td_count = lst->td_count;
  int odd = lst->odd;

  do {
    mfc_getllar(sync, sync_ea, 0, 0);
    mfc_read_atomic_status();
    for (j=0; j<(LIST_COUNT+3)/4; j++) {
      c[j] = sync[j];
      sync[j] = c[j] + td_count[j];
    }
    mfc_putllc(sync, sync_ea, 0, 0);
    status = mfc_read_atomic_status() & MFC_PUTLLC_STATUS;
  } while (status);

  for (j=0; j<LIST_COUNT; j++) {
    dst_ea = d3d.td_ea[j] + spu_extract(c[j/4], j&3)*128;
    mfc_put(lst->td+(odd*LIST_COUNT+j)*MAX_TD_COUNT, dst_ea,
        spu_extract(td_count[j/4], j&3)*128, odd+2, 0, 0);
  }

  for (j=0; j<(LIST_COUNT+3)/4; j++)
    td_count[j] = spu_splats(0);
  lst->odd = 1-odd;
  wait_for_completion(3-odd);
}

void rot_trans_proj_render_triangle2d(vec_ushort8 t0,
    vec_float4 va, vec_float4 vb, vec_float4 vc,
    vec_float4 na, vec_float4 nb, vec_float4 nc, struct list_buf *lst) {
  int j, jmax, tst;
  vec_float4 x, y, z, l, u, v;
  vec_uint4 csuv, sh_type;
  vec_float4 dx, dy, dz, dl, du, dv, dy1, q1, dxr, dzr, dlr, dur, dvr, y_;
  vec_float4 zl_l, dzl_l, uv_l, duv_l, ymmzl;
  qword ord;
  vec_uchar16 s_min, s_max, shuf, s_xzr;
  vec_float4 ymin, ymax, xr, zr, lr, ur, vr, r0l0, r1l1;
  vec_float4 xlr, zlr, llr, ulr, vlr, w, w1;
  vec_float4 zili, uivi, yminmax;
  vec_int4 yminmax_i;
  triangle_data *tdp;
  vec_int4 tdc;
  vec_int4 *td_count = lst->td_count;
  triangle_data *td = lst->td + lst->odd*MAX_TD_COUNT*LIST_COUNT;

  const vec_uchar16 v_2367 = (vec_uchar16) { S_2, S_3, S_6, S_7 };
  const vec_float4 hh = (vec_float4) { 0.5f, 0.5f, 0.5f, 0.5f };
  const vec_float4 light = d3d.lightsource;
  const vec_float4 zero = (vec_float4) { 0.f, 0.f, 0.f, 0.f };
  const vec_float4 one = (vec_float4) { 1.f, 1.f, 1.f, 1.f };
  const vec_float4 intfy = (vec_float4) { 1e+20f, 1e+20f, 1e+20f, 1e+20f };
  /*const vec_float4 heightw = spu_insert((float)sped.height, spu_splats(0.f),
  1);*/

  sh_type = (vec_uint4)spu_shuffle(t0, t0, (vec_uchar16)spu_splats(0x80800203));
  uint32_t ish_type = spu_extract(t0, 1);
  l = u = v = zero;
  csuv = (vec_uint4)zero;
  /* lighting computation for na, nb, nc vectors */
  switch (ish_type) {
  case SH_FLAT: {
  /* Flat shading with specular lighting */
  vec_float4 id = spu_convtf((vec_uint4)spu_shuffle(t0, t0,
    ((vec_uchar16) { 0x80, 0x80, 0x80, 13, 0x80, 0x80, 0x80, 14,
                     0x80, 0x80, 0x80, 15, 0x80, 0x80, 0x80, 0x80 })), 0);
  static const vec_float4 is = (const vec_float4){255.f, 255.f, 255.f, 0.f};
  vec_float4 kd = spu_splats(1.0f);
  vec_float4 ks = spu_splats(.7f);
  vec_float4 max = spu_splats(255.f);
  vec_float4 lmn = na * light;
  lmn = spu_shuffle(lmn,lmn,S_0000) + spu_shuffle(lmn,lmn,S_1111)
        + spu_shuffle(lmn,lmn,S_2222);
  vec_float4 ref_light = spu_splats(2.f) * lmn * na - light;
  vec_float4 rmv = -spu_shuffle(ref_light, ref_light, S_2222);
  rmv = (vec_float4)(((vec_uint4)rmv) & spu_cmpgt(rmv, spu_splats(0.f)));
  rmv *= rmv;
  rmv *= rmv;
  rmv *= rmv;
  vec_float4 vcolor = kd*lmn*id + ks*rmv*is;
  vec_uint4 icolor = spu_convtu(spu_sel(vcolor, max, spu_cmpgt(vcolor, max)),0);
  csuv = spu_shuffle(icolor, (vec_uint4)t0, ((vec_uchar16)
      {0x80, 3, 7, 11, 0x80, 0x80, 18, 19, S_E, S_E}));
  }
  break;
  case SH_SMOOTH: {
  /* Gouraud shading with specular lighting */
  vec_float4 id = spu_convtf((vec_uint4)spu_shuffle(t0, t0,
    ((vec_uchar16) { 0x80, 0x80, 0x80, 13, 0x80, 0x80, 0x80, 14,
                     0x80, 0x80, 0x80, 15, 0x80, 0x80, 0x80, 0x80 })), 8);
  const vec_float4 ks = spu_splats(.7f);
  const vec_float4 max = spu_splats(.999f);
  vec_float4 rmv, sc, sca, scb, scc, nz;

  sc = na * light;
  sca = sc + spu_shuffle(sc,sc,S_1___) + spu_shuffle(sc,sc,S_2___);
  sc = nb * light;
  scb = sc + spu_shuffle(sc,sc,S_1___) + spu_shuffle(sc,sc,S_2___);
  sc = nc * light;
  scc = sc + spu_shuffle(sc,sc,S_1___) + spu_shuffle(sc,sc,S_2___);
  sc = spu_shuffle(spu_shuffle(sca, scb, S_04__), scc, S_014_);

  l = spu_shuffle(id, id, S_0000) * sc;
  u = spu_shuffle(id, id, S_1111) * sc;
  v = spu_shuffle(id, id, S_2222) * sc;

  nz = spu_shuffle(spu_shuffle(na, nb, S_26__), nc, S_016_);
  rmv = -(spu_splats(2.f) * sc * nz - spu_shuffle(light, light, S_2222));
  rmv = (vec_float4)(((vec_uint4)rmv) & spu_cmpgt(rmv, spu_splats(0.f)));
  rmv *= rmv;
  rmv *= rmv;
  rmv *= rmv;
  l += ks*rmv;
  u += ks*rmv;
  v += ks*rmv;
  l = spu_sel(l, max, spu_cmpgt(l, max));
  u = spu_sel(u, max, spu_cmpgt(u, max));
  v = spu_sel(v, max, spu_cmpgt(v, max));
  csuv = spu_shuffle((vec_uint4)t0, sh_type, S_34__);
  }
  break;
  case SH_RGBSMOOTH:
  /* RGB gouraud shading */
  l = spu_shuffle(na, nb, S_04__);
  u = spu_shuffle(na, nb, S_15__);
  v = spu_shuffle(na, nb, S_26__);
  l = spu_shuffle(l, nc, S_014_) * hh + hh;
  u = spu_shuffle(u, nc, S_015_) * hh + hh;
  v = -spu_shuffle(v, nc, S_016_) * hh + hh;
  csuv = spu_shuffle((vec_uint4)t0, sh_type, S_34__);
  }

  /*** Now generate triangle data ***/
  x = spu_shuffle(va, vb, S_04__);
  y = spu_shuffle(va, vb, S_15__);
  z = spu_shuffle(va, vb, S_26__);
  x = spu_shuffle(x, vc, S_014_);
  y = spu_shuffle(y, vc, S_015_);
  z = spu_shuffle(z, vc, S_016_);
  l = l*z;
  u = u*z;
  v = v*z;

  /* Order x's, y's, and z's such that y0 is lowest */
  ord = (qword)spu_rl(spu_gather(spu_cmpgt(y,spu_shuffle(y,y,S_1201))),3);
  s_min = (vec_uchar16)si_lqx(si_from_ptr(lut_min),ord);
  y_ = y;
  x = spu_shuffle(x, x, s_min);
  y = spu_shuffle(y, y, s_min);
  z = spu_shuffle(z, z, s_min);
  l = spu_shuffle(l, l, s_min);
  u = spu_shuffle(u, u, s_min);
  v = spu_shuffle(v, v, s_min);

  /* Calculate triangle x, z and light differentials */
  dx = spu_shuffle(x,x,S_1201) - x;
  dy = spu_shuffle(y,y,S_1201) - y;
  dz = spu_shuffle(z,z,S_1201) - z;
  dl = spu_shuffle(l,l,S_1201) - l;
  du = spu_shuffle(u,u,S_1201) - u;
  dv = spu_shuffle(v,v,S_1201) - v;
  dy1 = spu_re(dy);

  /* Increase Reciprocal precision */
  q1 = spu_nmsub(dy, dy1, one);
  dy1 = spu_madd(dy1,q1,dy1);
  dy1 = spu_sel(dy1, intfy, spu_cmpeq(dy, zero));

  dxr = dx * dy1;
  dzr = dz * dy1;
  dlr = dl * dy1;
  dur = du * dy1;
  dvr = dv * dy1;

  /* Calculate y min and max */
  ymin = spu_shuffle(y, y, S_0000);
  s_max = (vec_uchar16)si_lqx(si_from_ptr(lut_max), ord);
  ymax = spu_shuffle(y_, y_, s_max);

  /* Calculate xr and zr */
  xr = x - dxr * (y-ymin);
  zr = z - dzr * (y-ymin);
  lr = l - dlr * (y-ymin);
  ur = u - dur * (y-ymin);
  vr = v - dvr * (y-ymin);

  r0l0 = spu_shuffle(xr, xr, S___02);
  r1l1 = spu_shuffle(xr, xr, S___13); 
  shuf = minlmaxr[si_to_int((qword)spu_gather(spu_cmpgt(r0l0, r1l1)))];
  xlr = spu_shuffle(xr, xr, shuf);
  zlr = spu_shuffle(zr, zr, shuf);
  llr = spu_shuffle(lr, lr, shuf);
  ulr = spu_shuffle(ur, ur, shuf);
  vlr = spu_shuffle(vr, vr, shuf);

  w = spu_shuffle(xlr, xlr, S_1111)-spu_shuffle(xlr, xlr, S_0000);
  w1 = spu_re(w);

  /* calculate constant z and l increments by pixel */
  zili = (spu_shuffle(zlr, llr, S_15__) - spu_shuffle(zlr, llr, S_04__)) * w1;
  uivi = (spu_shuffle(ulr, vlr, S_15__) - spu_shuffle(ulr, vlr, S_04__)) * w1;

  /* Adapt xr and zr */
  s_xzr = lut_ad[si_to_int((qword)spu_gather(
      spu_cmpgt(spu_shuffle(y, y, S__112), spu_shuffle(y, y, S__021))))];
  xr = spu_shuffle(xr, xr, s_xzr);
  zr = spu_shuffle(zr, zr, s_xzr);
  lr = spu_shuffle(lr, lr, s_xzr);
  ur = spu_shuffle(ur, ur, s_xzr);
  vr = spu_shuffle(vr, vr, s_xzr);
  dxr = spu_shuffle(dxr, dxr, s_xzr);
  dzr = spu_shuffle(dzr, dzr, s_xzr);
  dlr = spu_shuffle(dlr, dlr, s_xzr);
  dur = spu_shuffle(dur, dur, s_xzr);
  dvr = spu_shuffle(dvr, dvr, s_xzr);
  zl_l = spu_shuffle(zr, lr, v_2367);
  dzl_l = spu_shuffle(dzr, dlr, v_2367);
  uv_l = spu_shuffle(ur, vr, v_2367);
  duv_l = spu_shuffle(dur, dvr, v_2367);
  yminmax = spu_shuffle(ymin, ymax, S_04__);

  ymmzl = spu_shuffle(yminmax, zili, S_0145);
  csuv = spu_shuffle(csuv, (vec_uint4)uivi, S_0145);

  vec_float4 rnd = (vec_float4) { .5001f, -.5001f, 0.f, 0.f };
  vec_float4 rnd2 = (vec_float4) { .0001f, .0001f, 0.f, 0.f };

  yminmax_i = spu_convts(yminmax+rnd,0) & spu_splats(-NSCAN);
  yminmax_i = spu_convts(spu_convtf(yminmax_i,0)*array_coef+rnd2,0);
  j = si_to_int((qword)yminmax_i);
  j &= -(j>=0);
  jmax = spu_extract(yminmax_i, 1);
  jmax = (jmax>=LIST_COUNT) ? LIST_COUNT-1 : jmax;

  tst = 0;
  while (j<=jmax) {
    tdc = td_count[j/4];
    tdp = td + j*MAX_TD_COUNT + spu_extract(tdc, j&3);
    tdc = tdc + spu_insert(1, spu_splats(0), j&3);
    tst+=si_to_int((qword)spu_gather(spu_cmpeq(tdc,spu_splats(MAX_TD_COUNT))));
    td_count[j/4] = tdc;
    j++;

    tdp->x_rl = xr;
    tdp->dx_rl = dxr;
    tdp->zl_l = zl_l;
    tdp->dzl_l = dzl_l;
    tdp->uv_l = uv_l;
    tdp->duv_l = duv_l;
    tdp->ymmzl = ymmzl;
    tdp->csuv = csuv;
  }
  if (tst)
    list_buf_flush(lst);
}

void proj_render_poly(vec_ushort8 t0, vec_float4 t1,
    vec_float4 a, vec_float4 b, vec_float4 c, vec_float4 d,
    vec_float4 na, vec_float4 nb, vec_float4 nc, vec_float4 nd,
    struct list_buf *lst) {
  const vec_float4 zs = (vec_float4) { 128.f, 128.f, 128.f, 1.f };
  const vec_float4 zoom = zoom_factor;
  const vec_int4 xc_i = spu_shuffle((vec_int4)si_from_int(sped.width),
      (vec_int4)si_from_int(sped.height), S_04__);
  const vec_float4 xcenter = spu_convtf(xc_i, 1);
  float x1, x2, y1, y2, vz;
  vec_float4 z1;
  uint16_t flags = spu_extract(t0, 4);
  int tst_flatn = flags & FL_FLATN;
  int tst_ds = flags & FL_DS;
  int tst_quad = (spu_extract(t0, 0) == OB_QUAD);

  if (tst_flatn)
    na = nb = nc = nd = normal(a,b,c);

  /* project a, b, c, d vectors */
  z1 = -spu_re(spu_shuffle(a, a, S_2222));
  a = a * zs * zoom * z1 + xcenter;
  a = spu_shuffle(a, z1, S_0163);
  z1 = -spu_re(spu_shuffle(b, b, S_2222));
  b = b * zs * zoom * z1 + xcenter;
  b = spu_shuffle(b, z1, S_0163);
  z1 = -spu_re(spu_shuffle(c, c, S_2222));
  c = c * zs * zoom * z1 + xcenter;
  c = spu_shuffle(c, z1, S_0163);
  z1 = -spu_re(spu_shuffle(d, d, S_2222));
  d = d * zs * zoom * z1 + xcenter;
  d = spu_shuffle(d, z1, S_0163);

  x1 = spu_extract(b, 0) - spu_extract(a, 0);
  x2 = spu_extract(c, 0) - spu_extract(b, 0);
  y1 = spu_extract(b, 1) - spu_extract(a, 1);
  y2 = spu_extract(c, 1) - spu_extract(b, 1);
  vz = x1*y2 - x2*y1;

  if (vz > 0.0f) {
    rot_trans_proj_render_triangle2d(t0, a, b, c, na, nb, nc, lst);
    if (tst_quad)
      rot_trans_proj_render_triangle2d(t0, c, d, a, nc, nd, na, lst);
  } else if (tst_ds) {
    rot_trans_proj_render_triangle2d(t0, a, c, b, -na, -nc, -nb, lst);
    if (tst_quad)
      rot_trans_proj_render_triangle2d(t0, c, a, d, -nc, -na, -nd, lst);
  }
}

static void rot_trans_proj_render_poly(vec_ushort8 t0, vec_float4 t1,
    vec_float4 a, vec_float4 b, vec_float4 c, vec_float4 d,
    vec_float4 na, vec_float4 nb, vec_float4 nc, vec_float4 nd,
    vec_float4 rotp, struct list_buf *lst) {
  vec_float4 rx, ry, rz, rt;
  vec_float4 x, y, z;

  int32_t itime = spu_extract((vec_int4)t1,0) - d3d.time;
  int32_t jtime = d3d.time - spu_extract((vec_int4)t1,3);
  uint16_t flags = spu_extract(t0, 4);
  uint16_t type = spu_extract(t0, 0);

  int tst_extr = flags & FL_EXTR;
  int tst_flatn = flags & FL_FLATN;
  uint32_t tst_quad = type == OB_QUAD;

  vec_float4 *rot = (vec_float4 *)&fd.rot[spu_extract(t0, 2)][0];

  rx = rot[0];
  ry = rot[1];
  rz = rot[2];
  rt = rot[3];
  /* rotate+translate a, b, c, d vectors */
  x = spu_shuffle(a, a, S_0000);
  y = spu_shuffle(a, a, S_1111);
  z = spu_shuffle(a, a, S_2222);
  a = rx*x + ry*y + rz*z + rt;
  x = spu_shuffle(b, b, S_0000);
  y = spu_shuffle(b, b, S_1111);
  z = spu_shuffle(b, b, S_2222);
  b = rx*x + ry*y + rz*z + rt;
  x = spu_shuffle(c, c, S_0000);
  y = spu_shuffle(c, c, S_1111);
  z = spu_shuffle(c, c, S_2222);
  c = rx*x + ry*y + rz*z + rt;
  x = spu_shuffle(d, d, S_0000);
  y = spu_shuffle(d, d, S_1111);
  z = spu_shuffle(d, d, S_2222);
  d = rx*x + ry*y + rz*z + rt;

  if (tst_extr) {
    /* Extrude */
    float thickness = (float)spu_extract(t0, 5);
    t0 = spu_insert(flags & ~FL_EXTR, t0, 4);
    vec_float4 vn = normal(a,b,c);
    vec_float4 v = vn * spu_splats(thickness * 0.5f);
    vec_float4 e = a + v;
    vec_float4 f = b + v;
    vec_float4 g = c + v;
    vec_float4 h = d + v;
    a = a - v;
    b = b - v;
    c = c - v;
    d = d - v;
    proj_render_poly(t0,t1,a,b,c,d,vn,vn,vn,vn,lst);
    proj_render_poly(t0,t1,g,f,e,h,-vn,-vn,-vn,-vn,lst);
    t0 = spu_insert(OB_QUAD, t0, 0);
    vn = normal(a,e,f);
    proj_render_poly(t0,t1,a,e,f,b,vn,vn,vn,vn,lst);
    vn = normal(b,f,g);
    proj_render_poly(t0,t1,b,f,g,c,vn,vn,vn,vn,lst);
    if (tst_quad) {
      vn = normal(c,g,h);
      proj_render_poly(t0,t1,c,g,h,d,vn,vn,vn,vn,lst);
      vn = normal(d,h,e);
      proj_render_poly(t0,t1,d,h,e,a,vn,vn,vn,vn,lst);
    } else {
      vn = normal(c,g,e);
      proj_render_poly(t0,t1,c,g,e,a,vn,vn,vn,vn,lst);
    }
    return;
  }

  if (!tst_flatn) {
    /* rotate na, nb, nc, nd vectors */
    x = spu_shuffle(na, na, S_0000);
    y = spu_shuffle(na, na, S_1111);
    z = spu_shuffle(na, na, S_2222);
    na = rx*x + ry*y + rz*z;
    x = spu_shuffle(nb, nb, S_0000);
    y = spu_shuffle(nb, nb, S_1111);
    z = spu_shuffle(nb, nb, S_2222);
    nb = rx*x + ry*y + rz*z;
    x = spu_shuffle(nc, nc, S_0000);
    y = spu_shuffle(nc, nc, S_1111);
    z = spu_shuffle(nc, nc, S_2222);
    nc = rx*x + ry*y + rz*z;
    x = spu_shuffle(nd, nd, S_0000);
    y = spu_shuffle(nd, nd, S_1111);
    z = spu_shuffle(nd, nd, S_2222);
    nd = rx*x + ry*y + rz*z;
  }
  proj_render_poly(t0,t1,a,b,c,d,na,nb,nc,nd,lst);
}

void rot_trans_proj_render_poly_packed(vec_float4 *tr, struct list_buf *lst) {
  vec_ushort8 t0 = (vec_ushort8)tr[0];
  vec_float4 t1 = tr[1];
  vec_float4 t2 = tr[2];
  vec_float4 t3 = tr[3];
  vec_float4 t4 = tr[4];
  vec_float4 t5 = tr[5];
  vec_short8 t6 = (vec_short8)tr[6];
  vec_short8 t7 = (vec_short8)tr[7];
  vec_float4 na, nb, nc, nd, rt;
  vec_float4 t6_0, t6_1, t7_0, t7_1;

  /* reconstruct normal na, nb, nc, nd vectors */
  t6_0 = spu_convtf(spu_extend(spu_rlqwbyte(t6,14)),15);
  t6_1 = spu_convtf(spu_extend(t6),15);
  t7_0 = spu_convtf(spu_extend(spu_rlqwbyte(t7,14)),15);
  t7_1 = spu_convtf(spu_extend(t7),15);
  na = spu_shuffle(t6_0, t6_1, S_0415);
  nb = spu_shuffle(t6_0, t6_1, S_2637);
  nc = spu_shuffle(t7_0, t7_1, S_0415);
  nd = spu_shuffle(t7_0, t7_1, S_2637);
  rt = spu_shuffle(t6_1, t7_1, S_1357);
  rot_trans_proj_render_poly(t0, t1, t2, t3, t4, t5, na, nb, nc, nd, rt, lst);
}

typedef void (*obj_manager)(vec_float4 *tr, struct list_buf *lst);

void rotate_translate_project_render(void) {
  const static obj_manager manager[] = { NULL,
      rot_trans_proj_render_poly_packed,
      rot_trans_proj_render_poly_packed,
  };
  char tr_[TR_COUNT*128*2+127];
  char td_[MAX_TD_COUNT*LIST_COUNT*128*2+127];
  struct list_buf lst;
  vec_float4 *trp = (vec_float4*)((((uint32_t)tr_)+127)&-128);
  vec_float4 *tr;
  int i, j;
  int odd = 0;
  uint16_t ob_type;
  vec_int4 count, nt, nt2, nmax, dec;
  uint64_t src_ea = d3d.t_ea;

  list_buf_init(&lst, (triangle_data*)((((uint32_t)td_)+127)&-128));
  i = sped.spe_rank*TR_COUNT;
  src_ea += i*128;
  count = (vec_int4)si_from_int(d3d.n_t - i);
  nmax = (vec_int4)si_from_int(TR_COUNT);
  dec = (vec_int4)si_from_int(TR_COUNT*sped.spe_count);
  count = count & spu_cmpgt(count, 0);
  nt = spu_sel(count, nmax, spu_cmpgt(count, nmax));
  mfc_get(trp, src_ea, si_to_int((qword)nt)*128, odd, 0, 0);
  count -= dec;

  odd = 1;
  while (i < d3d.n_t) {
    i += TR_COUNT*sped.spe_count;
    src_ea += si_to_int((qword)nt)*128*sped.spe_count;
    nt2 = spu_sel(count, nmax, spu_cmpgt(count, nmax)) & spu_cmpgt(count, 0);
    mfc_get(trp+odd*8*TR_COUNT, src_ea, si_to_int((qword)nt2)*128, odd, 0, 0);
    count -= dec;

    odd = 1-odd;
    wait_for_completion(odd);

    tr = trp+odd*8*TR_COUNT;
    for (j=0; j<si_to_int((qword)nt); j++, tr+=8) {
      ob_type = spu_extract((vec_ushort8)tr[0], 0);
      manager[ob_type](tr, &lst);
    }

    nt = nt2;
  }

  int n_shared = d3d.n_shared;
  if (n_shared) {
    mfc_get(trp, d3d.sh_t_ea, n_shared*128, 0, 0, 0);
    tr = trp;
    wait_for_completion(0);
    for (j=0; j<n_shared; j++) {
      ob_type = spu_extract((vec_ushort8)tr[0], 0);
      manager[ob_type](tr, &lst);
    }
  }

  list_buf_flush(&lst);
  wait_for_completion(3-lst.odd);
}

#define BLUR_BLOCKSIZE 16384
void motion_blur(void) {
  char buf_[BLUR_BLOCKSIZE*4+127];
  vec_uint4 *buf = (vec_uint4*)((((uint32_t)buf_)+127)&-128);
  vec_uint4 *buf0 = buf;
  vec_uint4 *buf1 = buf + 2*(BLUR_BLOCKSIZE/16);
  vec_float4 blur0 = spu_splats(fd.blur_strength);
  vec_float4 blur1 = spu_splats(fd.blur_strength1);
  vec_float4 flash = spu_splats(fd.flash_strength);
  vec_float4 max = spu_splats(255.f);
  vec_int4 fc = (vec_int4)si_from_int(fd.filter_color);
  vec_float4 filter_strength = spu_splats(fd.filter_strength);
  vec_float4 filter_strength1 = spu_splats(1.f) - filter_strength;
  vec_float4 filter_r = spu_convtf(spu_shuffle(fc, fc,
          ((vec_uchar16) {S_E3, 1, S_E3, 1, S_E3, 1, S_E3, 1})), 0);
  vec_float4 filter_g = spu_convtf(spu_shuffle(fc, fc,
          ((vec_uchar16) {S_E3, 2, S_E3, 2, S_E3, 2, S_E3, 2})), 0);
  vec_float4 filter_b = spu_convtf(spu_shuffle(fc, fc,
          ((vec_uchar16) {S_E3, 3, S_E3, 3, S_E3, 3, S_E3, 3})), 0);

  /*vec_float4 abscmp = spu_splats(3.f);*/
  vec_uint4 a, r, g, b;
  vec_float4 r0, g0, b0, r1, g1, b1;
  int32_t bs, old_bs;
  uint64_t oldscreen_ea = fd.oldscreen_ea;
  if (oldscreen_ea == 0)
    oldscreen_ea = fd.screen_ea;
  uint64_t ea0 = fd.screen_ea + sped.spe_rank * BLUR_BLOCKSIZE;
  uint64_t ea1 = oldscreen_ea + sped.spe_rank * BLUR_BLOCKSIZE;
  uint64_t ea_end = fd.screen_ea + sped.pitch * sped.height;
  uint32_t ea_inc = sped.spe_count * BLUR_BLOCKSIZE;
  int32_t remain = ea_end - ea0;
  uint64_t ea2, ea3;
  int odd = 0;
  int i;

  mfc_get(buf0, ea0, BLUR_BLOCKSIZE, odd, 0, 0);
  mfc_get(buf1, ea1, BLUR_BLOCKSIZE, odd, 0, 0);
  odd = 1;
  old_bs = BLUR_BLOCKSIZE;
  while (remain > 0) {
    ea2 = ea0 + ea_inc;
    ea3 = ea1 + ea_inc;
    remain = ea_end - ea2;
    bs = SPU_MIN_INT(BLUR_BLOCKSIZE, remain);

    bs = SPU_MAX0_INT(bs);
    buf = buf0 + ((-odd)&(BLUR_BLOCKSIZE/16));
    mfc_getb(buf, ea2, bs, odd, 0, 0);
    buf = buf1 + ((-odd)&(BLUR_BLOCKSIZE/16));
    mfc_getb(buf, ea3, bs, odd, 0, 0);

    odd = 1-odd;
    wait_for_completion(odd);

    /* Perform the blur stuff */
    buf = buf0 + odd*(BLUR_BLOCKSIZE/16);
    vec_uint4 *buf_ = buf1 + odd*(BLUR_BLOCKSIZE/16);
    for (i = 0; i < old_bs/16; i++) {
      a = buf[i];
      b = buf_[i];
      r0 = spu_convtf(spu_shuffle(a, a,
          ((vec_uchar16) {S_E3, 1, S_E3, 5, S_E3, 9, S_E3, 13})), 0);
      g0 = spu_convtf(spu_shuffle(a, a,
          ((vec_uchar16) {S_E3, 2, S_E3, 6, S_E3, 10, S_E3, 14})), 0);
      b0 = spu_convtf(spu_shuffle(a, a,
          ((vec_uchar16) {S_E3, 3, S_E3, 7, S_E3, 11, S_E3, 15})), 0);
      r1 = spu_convtf(spu_shuffle(b, b,
          ((vec_uchar16) {S_E3, 1, S_E3, 5, S_E3, 9, S_E3, 13})), 0);
      g1 = spu_convtf(spu_shuffle(b, b,
          ((vec_uchar16) {S_E3, 2, S_E3, 6, S_E3, 10, S_E3, 14})), 0);
      b1 = spu_convtf(spu_shuffle(b, b,
          ((vec_uchar16) {S_E3, 3, S_E3, 7, S_E3, 11, S_E3, 15})), 0);
      r1 = blur0 * r0 + blur1 * r1 + flash;
      g1 = blur0 * g0 + blur1 * g1 + flash;
      b1 = blur0 * b0 + blur1 * b1 + flash;
      r1 = spu_sel(r1, max, spu_cmpgt(r1, max));
      g1 = spu_sel(g1, max, spu_cmpgt(g1, max));
      b1 = spu_sel(b1, max, spu_cmpgt(b1, max));
      r1 = r1 * filter_strength1 + filter_r * filter_strength;
      g1 = g1 * filter_strength1 + filter_g * filter_strength;
      b1 = b1 * filter_strength1 + filter_b * filter_strength;
      /*r1 = spu_sel(r0, r1, spu_cmpabsgt((r0-r1), abscmp));
      g1 = spu_sel(g0, g1, spu_cmpabsgt((g0-g1), abscmp));
      b1 = spu_sel(b0, b1, spu_cmpabsgt((b0-b1), abscmp));*/
      r = spu_slqwbyte(spu_convtu(r1,0),2);
      g = spu_slqwbyte(spu_convtu(g1,0),1);
      b = spu_convtu(b1,0);
      buf[i] = r | g | b;
    }
    buf = buf0 + ((-odd)&(BLUR_BLOCKSIZE/16));
    mfc_put(buf, ea0, old_bs, odd, 0, 0);
    ea0 = ea2;
    ea1 = ea3;
    old_bs = bs;
  }
  wait_for_completion(odd);
  wait_for_completion(1-odd);
}

int main(uint64_t spe_id, uint64_t data_ea)
{
  uint64_t d3d_ea;
  uint32_t eah, eal;
  vec_float4 a, b, x0, q0, q1, q2, err;

  /* read Data structure from main memory */
  mfc_get(&sped, data_ea, sizeof(sped), DMA_TAG, 0, 0);
  wait_for_completion(0);

  /* get ea of 3D effect data */
  eah = spu_read_in_mbox();
  eal = spu_read_in_mbox();
  d3d_ea = ((uint64_t)eah)<<32 | eal;

  a = spu_splats((float)LIST_COUNT);
  b = spu_convtf(spu_splats(sped.height), 0);
  x0 = spu_re(b);
  q0 = spu_mul(a, x0);
  q1 = spu_nmsub(b, q0, a);
  q1 = spu_madd(x0, q1, q0);
  q2 = (vec_float4)spu_add((vec_uint4)(q1), 1);
  err = spu_nmsub(b, q2, a);
  q2 = spu_sel(q1, q2, spu_cmpgt((vec_int4)err, -1));
  array_coef = q2;

  zoom_factor = spu_splats((float)sped.height * (1.0f/160.f))
      * ((vec_float4){1.0f, -1.0f, 1.0f, 0.0f})
      + ((vec_float4){0.0f, 0.0f, 0.0f, 1.0f});

  /* send ready message */
  spu_write_out_mbox(0);

  /* main loop */
  while (spu_read_in_mbox() != 0)
  {
    mfc_get(&d3d, d3d_ea, sizeof(d3d), 0, 0, 0);
    wait_for_completion(0);
    mfc_get(&fd, d3d.fd_ea, sizeof(spe_frame_data), 0, 0, 0);
    wait_for_completion(0);

    rotate_translate_project_render();
    spe_barrier();
    if (sped.spe_rank == 0)
      spu_write_out_mbox(0);

    draw_screen();
    spe_barrier();
    if (fd.blur_strength >= 0.f) {
      motion_blur();
      spe_barrier();
    }
    if (sped.spe_rank == 0)
      spu_write_out_mbox(0);
  }

  return 0;
}

