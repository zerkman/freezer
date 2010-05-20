/* Miscellaneous data structures and functions.
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

#define USE_ALTIVEC

#ifdef USE_ALTIVEC
#include "cos_sin18_v.h"
#else
#include <math.h>
#endif

#include "frz_misc.hpp"

#define S_0 0x00, 0x01, 0x02, 0x03
#define S_1 0x04, 0x05, 0x06, 0x07
#define S_2 0x08, 0x09, 0x0a, 0x0b
#define S_3 0x0c, 0x0d, 0x0e, 0x0f
#define S_4 0x10, 0x11, 0x12, 0x13
#define S_5 0x14, 0x15, 0x16, 0x17
#define S_6 0x18, 0x19, 0x1a, 0x1b
#define S_7 0x1c, 0x1d, 0x1e, 0x1f

#define S_0400 (vector unsigned char) { S_0, S_4, S_0, S_0 }
#define S_0140 (vector unsigned char) { S_0, S_1, S_4, S_0 }

namespace Frz {

/* Load a float vector from a potentially unaligned address */
vector float load_unaligned(const float *target) {
  vector float msq, lsq;
  vector unsigned char mask;
  msq = vec_ld(0, target);               // most significant quadword
  lsq = vec_ld(15, target);              // least significant quadword
  mask = vec_lvsl(0, target);            // create the permute mask
  return vec_perm(msq, lsq, mask);       // align the data
}

/* Store a float vector to a potentially unaligned address */
void store_unaligned(float *target, vector float src) {
  vector float msq, lsq, edges;
  vector unsigned char edgeAlign, align;
  msq = vec_ld(0, target);               // most significant quadword
  lsq = vec_ld(15, target);              // least significant quadword
  edgeAlign = vec_lvsl(0, target);       // permute map to extract edges
  edges = vec_perm(lsq, msq, edgeAlign); // extract the edges
  align = vec_lvsr(0, target);           // permute map to misalign data
  msq = vec_perm(edges, src, align);     // misalign the data (msq)
  lsq = vec_perm(src, edges, align);     // misalign the data (lsq)
  vec_st(lsq, 15, target);               // Store the lsq part first
  vec_st(msq, 0, target);                // Store the msq part
}

/* create a rotation and translation matrix (columnwise matrix) */
#define CNV_ANGL (16/3.1415927f)
#define V_CNV_ANGL (vector float){CNV_ANGL, CNV_ANGL, CNV_ANGL, CNV_ANGL}
void make_rotation(vector float rot[4], trans &t) {
#ifdef USE_ALTIVEC
  vector float rotation = load_unaligned(&t.a) * V_CNV_ANGL;
  vector float translation = load_unaligned(&t.x);
  vector float sin = _cos_sin18_v(rotation - (vector float){8.f,8.f,8.f,8.f});
  vector float cos = _cos_sin18_v(rotation);
  vector float sin_a = sin;
  vector float sin_b = vec_sld(sin, sin, 4);
  vector float sin_c = vec_sld(sin, sin, 8);
  vector float cos_a = cos;
  vector float cos_b = vec_sld(cos, cos, 4);
  vector float cos_c = vec_sld(cos, cos, 8);
  //vector float zero = (vector float)vec_splat_s32(0);

  /* row 0 */
  vector float r00 = cos_b * cos_c;
  vector float r10 = -cos_b * sin_c;
  vector float r20 = sin_b;
  /* row 1 */
  vector float r01 = sin_a * sin_b * cos_c + cos_a * sin_c;
  vector float r11 = -sin_a * sin_b * sin_c + cos_a * cos_c;
  vector float r21 = -sin_a * cos_b;
  /* row 2 */
  vector float r02 = -cos_a * sin_b * cos_c + sin_a * sin_c;
  vector float r12 = cos_a * sin_b * sin_c + sin_a * cos_c;
  vector float r22 = cos_a * cos_b;

  rot[0] = vec_perm(vec_perm(r00, r01, S_0400), r02, S_0140);
  rot[1] = vec_perm(vec_perm(r10, r11, S_0400), r12, S_0140);
  rot[2] = vec_perm(vec_perm(r20, r21, S_0400), r22, S_0140);
//rot[0] = vec_perm(vec_perm(vec_perm(r00,r01,S_0400),r02,S_0140),zero,S_0124);
//rot[1] = vec_perm(vec_perm(vec_perm(r10,r11,S_0400),r12,S_0140),zero,S_0124);
//rot[2] = vec_perm(vec_perm(vec_perm(r20,r21,S_0400),r22,S_0140),zero,S_0124);
  rot[3] = translation;
#else
  float *rot_ = (float *)rot;
  memset(rot, 0, 16*sizeof(float));
  float sin_a = sinf(t.a);
  float cos_a = cosf(t.a);
  float sin_b = sinf(t.b);
  float cos_b = cosf(t.b);
  float sin_c = sinf(t.c);
  float cos_c = cosf(t.c);
  /* row 0 */
  rot_[0*4 + 0] = cos_b * cos_c;
  rot_[1*4 + 0] = -cos_b * sin_c;
  rot_[2*4 + 0] = sin_b;
  /* row 1 */
  rot_[0*4 + 1] = sin_a * sin_b * cos_c + cos_a * sin_c;
  rot_[1*4 + 1] = -sin_a * sin_b * sin_c + cos_a * cos_c;
  rot_[2*4 + 1] = -sin_a * cos_b;
  /* row 2 */
  rot_[0*4 + 2] = -cos_a * sin_b * cos_c + sin_a * sin_c;
  rot_[1*4 + 2] = cos_a * sin_b * sin_c + sin_a * cos_c;
  rot_[2*4 + 2] = cos_a * cos_b;
  /* Translation coefs */
  rot_[3*4 + 0] = t.x;
  rot_[3*4 + 1] = t.y;
  rot_[3*4 + 2] = t.z;
  rot_[3*4 + 3] = 1.0f;
#endif
}

}; // namespace Frz;
