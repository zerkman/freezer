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

struct trans {
  float a, b, c, d;
  float x, y, z, w;
};

/* Load a float vector from a potentially unaligned address */
vector float load_unaligned(const float *target);

/* Store a float vector to a potentially unaligned address */
void store_unaligned(float *target, vector float src);

/* create a rotation and translation matrix (columnwise matrix) */
void make_rotation(vector float rot[4], trans &t);

/* class for aligned vector */
template <class type>
class alignvec {
  type *ptr;
  int align;
  int size;
  int capacity;
  bool growable;

public:
  alignvec() : ptr(0), align(128), size(0), capacity(0), growable(true) {}
  alignvec(int _align, int _capacity=0) :
      ptr(0), align(_align), size(0), capacity(0), growable(true) {
    if (_capacity)
      setFixedCapacity(_capacity);
  }
  virtual ~alignvec() {
    if (ptr)
      free(ptr);
  }

  void setFixedCapacity(int cap) {
    growup(cap);
    clear();
    growable = false;
  }

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

  void clear() {
    size = 0;
  }

  int getSize() const {
    return size;
  }
  int getCapacity() const {
    return capacity;
  }

  type & operator[](int i) {
    return ptr[i];
  }
  const type & operator[](int i) const  {
    return ptr[i];
  }
};

}; // namespace Frz;

#endif

