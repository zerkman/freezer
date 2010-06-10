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

/*! \brief A 3D vertex.
 *
 * This class generally is used for storing a set of 3D coordinates, but also
 * may embed extra data. Its main purpose is to be packed into
 * Scene::Triangle objects, for communication with the SPE's. In this case, a
 * vertex will be loaded into a 16-byte vector on SPU side.
 *
 * The fourth vertex element is generally ignored by the arithmetics and
 * normalization methods. It is used for storage and 16-byte alignment only.
 */
struct vertex {
  union {
    float x;     //!< First vertex coordinate in floating point format
    uint32_t a;  //!< First vertex coordinate in 32-bit integer format
  };
  union {
    float y;     //!< Second vertex coordinate in floating point format
    uint32_t b;  //!< Second vertex coordinate in 32-bit integer format
  };
  union {
    float z;     //!< Third vertex coordinate in floating point format
    uint32_t c;  //!< Third vertex coordinate in 32-bit integer format
  };
  union {
    float t;     //!< Fourth vertex coordinate in floating point format
    uint32_t s;  //!< Fourth vertex coordinate in 32-bit integer format
  };

  /*! \brief Default constructor.
   *
   * \param _x the first vertex coordinate
   * \param _y the second vertex coordinate
   * \param _z the third vertex coordinate
   * \param _t the fourth vertex coordinate
   */
  vertex(float _x=0.f, float _y=0.f, float _z=0.f, float _t=1.0f):
      x(_x), y(_y), z(_z), t(_t) {}

  /*! \brief Copy constructor.
   *
   * \param v the vertex to be copied
   */
  vertex(const vertex &v): x(v.x), y(v.y), z(v.z), t(v.t) {}

  /*! \brief Integer constructor.
   *
   * \param _a the first vertex coordinate
   * \param _b the second vertex coordinate
   * \param _c the third vertex coordinate
   * \param _s the fourth vertex coordinate
   */
  vertex(uint32_t _a, uint32_t _b, uint32_t _c, uint32_t _s):
      a(_a), b(_b), c(_c), s(_s) {}

  /*! \brief Altivec vector conversion constructor.
   *
   * \param v the Altivec vector variable to be converted
   */
  vertex(vector float v) { set(v); }

  /*! \brief Sets a vertex value from an Altivec vector.
   *
   * \param v the Altivec vector
   */
  void set(vector float v) { store_unaligned(&x, v); }

  /*! \brief vertex subtraction operator.
   *
   * \param a the vertex to be subtracted to this
   *
   * \return the result of the operation
   */
  vertex operator-(const vertex &a) const {return vertex(x-a.x, y-a.y, z-a.z);}

  /*! \brief vertex addition operator.
   *
   * \param a the vertex to be added to this
   *
   * \return the result of the operation
   */
  vertex operator+(const vertex &a) const {return vertex(x+a.x, y+a.y, z+a.z);}

  /*! \brief vertex cross product operator.
   *
   * \param b the vertex to be the right hand operand to the cross product
   *
   * \return the result of the operation
   */
  vertex operator*(const vertex &b) const {
    return vertex(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x);
  }

  /*! \brief Scale all coefficients with a scalar value.
   *
   * \param f the scalar value to be multiplied with all the vertex
   *          coordinates
   *
   * \return the result of the operation
   */
  vertex operator*(float f) const { return vertex(x*f, y*f, z*f); }

  /*! \brief Normalizes the vertex.
   *
   * All terms of the vertex are divided by the vertex's norm.
   */
  void normalize(void);

  /*! \brief Computes the square norm of the vertex.
   *
   * \return the square of the vertex's norm
   */
  float norm2(void);

  /*! \brief Computes the norm of the vertex.
   *
   * \return the vertex's norm
   */
  float norm(void);

  /*! \brief Updates the normal vector in vertex \p b of triangle (\p a,\p b,
   * \p c).
   *
   * This corresponds to adding the normalized <em>ab * bc</em> cross
   * product, multiplied by the angle in \p b
   */
  void update_norm(const vertex &a, const vertex &b, const vertex &c);

  /*! \brief Outputs the vertex in human-readable fashion.
   *
   * \param os the output stream
   * \param v the vertex to be displayed
   *
   * \return the output stream
   */
  friend std::ostream &operator<<(std::ostream &os, const vertex &v) {
    os <<"("<<v.x<<","<<v.y<<","<<v.z<<","<<v.t<<")";
    return os;
  }

  /*! \brief Converts the vertex to an Altivec vector.
   *
   * \return the converted Altivec vector
   */
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
