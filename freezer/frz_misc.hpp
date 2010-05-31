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

#ifndef __MISC_HPP__
#define __MISC_HPP__

#include <iostream>
#include <cstdlib>
#include <cstring>

#include <altivec.h>

//typedef unsigned int ptr_t;

#ifdef __powerpc__
#ifndef __powerpc64__
#define spu_ea(val) ((uint64_t) ((uint32_t) val))
#else
#define spu_ea(val) ((uint64_t) val)
#endif
#endif

namespace Frz {

//! A structure for storing rotation and translation coefficients.
struct trans {
  float a;  //!< Rotation angle on the \e x axis.
  float b;  //!< Rotation angle on the \e y axis.
  float c;  //!< Rotation angle on the \e z axis.
  float d;  //!< Unused.
  float x;  //!< Translation value on the \e x axis.
  float y;  //!< Translation value on the \e y axis.
  float z;  //!< Translation value on the \e z axis.
  float w;  //!< Unused.
};

/* Loads a float vector from a potentially unaligned address. */
vector float load_unaligned(const float *target);

/* Stores a float vector to a potentially unaligned address. */
void store_unaligned(float *target, vector float src);

/* Creates a rotation and translation matrix (columnwise matrix). */
void make_rotation(vector float rot[4], trans &t);

/*! \brief Memory-aligned resizable buffer.
 *
 * Implements a typed buffer with parametrizable alignment in memory, and
 * optional resizable feature.
 */
template <class type>
class alignvec {
  type *ptr;
  int align;
  int size;
  int capacity;
  bool growable;

public:
  /*! \brief Constructor.
   *
   * Allocates a buffer with specified alignment, and with the choice for fixed
   * size or not.
   *
   * \param _align Memory alignment in bytes.
   * \param _capacity
   *    - If zero, the buffer is resizable and has zero initial capacity.
   *    - Otherwise, the buffer has a fixed capacity of \p _capacity.
   */
  alignvec(int _align=128, int _capacity=0) :
      ptr(0), align(_align), size(0), capacity(0), growable(true) {
    if (_capacity)
      setFixedCapacity(_capacity);
  }
  //! Destructor.
  virtual ~alignvec() {
    if (ptr)
      free(ptr);
  }
  //! Sets the buffer capacity to \p cap, and makes it non-resizable.
  void setFixedCapacity(int cap) {
    growup(cap);
    clear();
    growable = false;
  }
  /*! \brief Increase the buffer size of \p count elements.
   *
   * \param count Buffer size increment.
   * \return The previous number of elements in the buffer.
   */
  int growup(int count = 1) {
    void *newptr;
    int oldsize;
    int newsize = size + count;
    if (newsize > capacity) {
      if (!growable) {
        std::cerr << "alignvec::growup() array is not growable (" << newsize
            << " > " << capacity << ")" << std::endl;
        exit(1);
      }
      if (capacity == 0)
        capacity = 1;
      while (newsize > capacity)
        capacity *= 2;
      if (posix_memalign(&newptr, align, capacity*sizeof(type)) != 0) {
        std::cerr << "alignvec::growup() allocation error" << std::endl;
        exit(1);
      }
      if (ptr) {
        memcpy(newptr, ptr, size*sizeof(type));
        free(ptr);
      }
      ptr = (type*)newptr;
    }
    oldsize = size;
    size = newsize;
    return oldsize;
  }
  //! Sets the buffer size to zero.
  void clear() {
    size = 0;
  }
  //! Returns the number of elements in the buffer.
  int getSize() const {
    return size;
  }
  //! Returns the allocated capacity of the buffer.
  int getCapacity() const {
    return capacity;
  }
  //! Returns element \p i in the buffer array.
  type & operator[](int i) {
    return ptr[i];
  }
  //! Returns element \p i in the buffer array.
  const type & operator[](int i) const  {
    return ptr[i];
  }
};

}; // namespace Frz;

#endif

