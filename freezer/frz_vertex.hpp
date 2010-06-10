/* Vertex classes.
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

#include <stdint.h>
#include <iostream>

#ifndef _FRZ_VERTEX_HPP_
#define _FRZ_VERTEX_HPP_

#include "frz_misc.hpp"

namespace Frz {

struct vertex {
  union { float x; uint32_t a; };
  union { float y; uint32_t b; };
  union { float z; uint32_t c; };
  union { float t; uint32_t s; };
  vertex(): x(0.f), y(0.f), z(0.f), t(0.f) {}
  vertex(const vertex &v): x(v.x), y(v.y), z(v.z), t(v.t) {}
  vertex(float _x, float _y, float _z, float _t=1.0f):
      x(_x), y(_y), z(_z), t(_t) {}
  vertex(uint32_t _a, uint32_t _b, uint32_t _c, uint32_t _s):
      a(_a), b(_b), c(_c), s(_s) {}
  vertex(vector float v) { set(v); }

  void set(vector float v) { store_unaligned(&x, v); }

  vertex operator-(const vertex &a) const {return vertex(x-a.x, y-a.y, z-a.z);}
  vertex operator+(const vertex &a) const {return vertex(x+a.x, y+a.y, z+a.z);}
  vertex operator*(const vertex &b) const {
    return vertex(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x);
  }
  vertex operator*(float f) const { return vertex(x*f, y*f, z*f); }

  void normalize(void);
  float norm2(void);
  float norm(void);
  void update_norm(const vertex &a, const vertex &b, const vertex &c);
  friend std::ostream &operator<<(std::ostream &os, const vertex &v) {
    os <<"("<<v.x<<","<<v.y<<","<<v.z<<","<<v.t<<")";
    return os;
  }
  vector float v() const { return load_unaligned(&x); }
};

/*! \brief Short vertex class for normalized vectors.
 *
 * All the vertex coordinates are stored in signed 16-bit fixed-point format
 * with 1 bit for the sign and 15 bits of mantissa. Encoded values hence may
 * only vary from -1 (\c 0x8000) to 0.99997 (0x7FFF). It can be considered
 * sufficient to store normal vectors.
 */
struct svertex {
  int16_t x;  //!< The \e x coordinate
  int16_t y;  //!< The \e y coordinate
  int16_t z;  //!< The \e z coordinate
  int16_t t;  //!< The \e t coordinate

  /*! \brief Default constructor.
   *
   * \param _x the \e x coordinate
   * \param _y the \e y coordinate
   * \param _z the \e z coordinate
   * \param _t the \e t coordinate
   */
  svertex(int16_t _x=0, int16_t _y=0, int16_t _z=0, int16_t _t=0x7fff)
      : x(_x), y(_y), z(_z), t(_t) {}

  /*! \brief Copy constructor.
   *
   * \param v the object to be copied.
   */
  svertex(const svertex &v): x(v.x), y(v.y), z(v.z), t(v.t) {}

  /*! \brief vertex-converting constructor.
   *
   * \param v the vertex object to be converted.
   */
  svertex(const vertex &v);
};

} // namespace Frz

#endif
