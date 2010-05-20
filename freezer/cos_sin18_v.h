/* --------------------------------------------------------------  */
/* (C)Copyright 2001,2007,                                         */
/* International Business Machines Corporation,                    */
/* Sony Computer Entertainment, Incorporated,                      */
/* Toshiba Corporation,                                            */
/*                                                                 */
/* All Rights Reserved.                                            */
/*                                                                 */
/* Redistribution and use in source and binary forms, with or      */
/* without modification, are permitted provided that the           */
/* following conditions are met:                                   */
/*                                                                 */
/* - Redistributions of source code must retain the above copyright*/
/*   notice, this list of conditions and the following disclaimer. */
/*                                                                 */
/* - Redistributions in binary form must reproduce the above       */
/*   copyright notice, this list of conditions and the following   */
/*   disclaimer in the documentation and/or other materials        */
/*   provided with the distribution.                               */
/*                                                                 */
/* - Neither the name of IBM Corporation nor the names of its      */
/*   contributors may be used to endorse or promote products       */
/*   derived from this software without specific prior written     */
/*   permission.                                                   */
/*                                                                 */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND          */
/* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,     */
/* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF        */
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE        */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR            */
/* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
/* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT    */
/* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)        */
/* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN       */
/* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR    */
/* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,  */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              */
/* --------------------------------------------------------------  */
/* PROLOG END TAG zYx                                              */
#ifndef _COS_SIN18_V_H_
#define _COS_SIN18_V_H_		1

#ifdef __SPU__
#include <spu_intrinsics.h>
#else /* not __SPU__ */
#include <altivec.h>
#endif


#ifndef M_16_PI
#define M_16_PI		 5.0929581789407         /* 16/pi */ 
#endif /* M_16_PI */

/*
 * FUNCTION
 *	vector float _cos_sin18_v(vector float angle)
 *
 * DESCRIPTION
 *	_cos_sin18_v computes the cosines of a vector of angle (expressed 
 *	in normalized radians) to an accuracy of at least 18 bits for 
 *	angle of 0.0 to 1.0. Angles outside this range are less accurate.
 *	Normalized radians mean that an angle of 1.0 corresponds to 2*PI 
 *	radians. The cosine is computed using an 8 segment piecewise, cubic
 *      approximation over the interval [0, 0.25) normalized radians.
 *	Symmetry of the cosine function is used to produce results over
 *	the remainder of the interval. The cubic evaluation is of the form 
 *		((A * x + B) * x + C) * x + D
 *	where x is the fractional input angle within the segments interval.
 */

/* Precomputed quadratic coefficients */
static vector float cos18_A_v[2] = {
  (vector float) { 0.00012354,0.00036588,0.00059416,0.00079961},
  (vector float) { 0.00097433,0.00111160,0.00120616,0.00125436} };

static vector float cos18_B_v[2] = {
  (vector float) { -0.01933826,-0.01896574,-0.01786437,-0.01607648},
  (vector float) { -0.01367078,-0.01073973,-0.00739595,-0.00376795} };

static vector float cos18_C_v[2] = {
  (vector float) { -0.00000000,-0.03830589,-0.07513972,-0.10908596},
  (vector float) { -0.13884009,-0.16325867,-0.18140332,-0.19257674} };

static vector float cos18_D_v[2] = {
  (vector float) { 1.00000000,0.98078525,0.92387950,0.83146960},
  (vector float) { 0.70710677,0.55557024,0.38268343,0.19509032} };


static __inline vector float _cos_sin18_v(vector float angle)
{
  vector unsigned int idx;
  vector unsigned int quadrant23;
  vector unsigned int negflag;
  vector float ctmpl, ctmph;
  vector float a, b, c, d;
  vector float ang, result;
  vector float fang;
  vector float ang2;
  vector unsigned int iang;
  vector unsigned int mirror_iang;
  vector float mirror_angf;

  ang = angle;
  
#ifdef __SPU__
  ang = (vector float)spu_rlmask(spu_sl((vector unsigned int)ang, 1), -1);

  iang  = spu_convtu(ang, 0);
  fang  = spu_convtf(iang, 0);
  ang   = spu_sub(ang, fang);

  /* Handle mirroring of the function */
  quadrant23 = spu_and(iang, 8);
  quadrant23 = spu_cmpeq(quadrant23, 8);

  mirror_iang = spu_xor(iang, 0x17);
  mirror_angf = spu_sub(spu_splats((float)1.0), ang);

  iang = spu_sel(iang, mirror_iang, quadrant23);
  ang  = spu_sel(ang, mirror_angf, quadrant23);

  /* Determine correct resultant sign */
  negflag  = spu_and(iang, 0x10);
  negflag  = spu_sl(negflag, 27);

  idx      = spu_and(iang, 7);
  idx      = spu_sl(idx, 2);
  idx      = spu_shuffle(idx, idx, ((vector unsigned char) { 0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x07, 0x07,
  								0x0b, 0x0b, 0x0b, 0x0b, 0x0f, 0x0f, 0x0f, 0x0f}));
  idx      = spu_sel(spu_splats((unsigned int)0x00010203), idx, spu_splats((unsigned int)0x1c1c1c1c));

  /* Fetch coeeficients */
  a = spu_shuffle(cos18_A_v[0], cos18_A_v[1], (vector unsigned char)(idx));
  b = spu_shuffle(cos18_B_v[0], cos18_B_v[1], (vector unsigned char)(idx));
  c = spu_shuffle(cos18_C_v[0], cos18_C_v[1], (vector unsigned char)(idx));
  d = spu_shuffle(cos18_D_v[0], cos18_D_v[1], (vector unsigned char)(idx));
			       
  ang2 = spu_mul(ang, ang);
  ctmpl = spu_madd(ang, c, d);
  ctmph = spu_madd(ang, a, b);
  result = spu_madd(ang2, ctmph, ctmpl);
  result = (vector float)spu_or((vector unsigned int)result, negflag);

#else
  ang  = vec_abs(ang);
  iang = vec_ctu(ang, 0);
  fang = vec_ctf(iang, 0);
  ang  = vec_sub(ang, fang);

  /* Handle mirroring of the function */
  quadrant23 = vec_and(iang, ((vector unsigned int) {8,8,8,8}));
  quadrant23 = (vector unsigned int)vec_cmpeq(quadrant23, ((vector unsigned int) {8,8,8,8}));

  mirror_iang = vec_xor(iang, ((vector unsigned int){0x17,0x17,0x17,0x17}));
  mirror_angf = vec_sub(((vector float) {1.0,1.0,1.0,1.0}), ang);

  iang = vec_sel(iang, mirror_iang, quadrant23);
  ang = (vector float)vec_sel((vector unsigned int)ang, (vector unsigned int)mirror_angf, quadrant23);

  /* Determine correct resultant sign */
  negflag = vec_and(iang, (((vector unsigned int) {0x10,0x10,0x10,0x10})));
  negflag = vec_sl(negflag, ((vector unsigned int) {27,27,27,27}));

  idx     = vec_and(iang, ((vector unsigned int) {7,7,7,7}));
  idx     = vec_sl(idx, ((vector unsigned int) {2,2,2,2}));
  idx     = vec_perm(idx, idx, ((vector unsigned char) { 0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x07, 0x07,
  							0x0b, 0x0b, 0x0b, 0x0b, 0x0f, 0x0f, 0x0f, 0x0f}));
  idx     = vec_sel(((vector unsigned int) {0x00010203,0x00010203,0x00010203,0x00010203}), idx,
  			((vector unsigned int) {0x1c1c1c1c,0x1c1c1c1c,0x1c1c1c1c,0x1c1c1c1c}));

  /* Fetch coeeficients */
  a = (vector float)vec_perm((vector unsigned int)(cos18_A_v[0]), (vector unsigned int)(cos18_A_v[1]), (vector unsigned char)(idx));
  b = (vector float)vec_perm((vector unsigned int)(cos18_B_v[0]), (vector unsigned int)(cos18_B_v[1]), (vector unsigned char)(idx));
  c = (vector float)vec_perm((vector unsigned int)(cos18_C_v[0]), (vector unsigned int)(cos18_C_v[1]), (vector unsigned char)(idx));
  d = (vector float)vec_perm((vector unsigned int)(cos18_D_v[0]), (vector unsigned int)(cos18_D_v[1]), (vector unsigned char)(idx));
			       
  ang2 = vec_madd(ang, ang, ((vector float) {0.0,0.0,0.0,0.0}));
  ctmpl = vec_madd(ang, c, d);
  ctmph = vec_madd(ang, a, b);
  result = vec_madd(ang2, ctmph, ctmpl);
  result = (vector float)vec_or((vector unsigned int)result, negflag);
#endif

  return (result);
}

#endif /* _COS_SIN18_V_H_ */





