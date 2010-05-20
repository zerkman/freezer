/* Pixel buffer manager.
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
/*
 * frz_pixbuf.hpp - pixel buffer manager for the Freezer animation system
 * by François Galea <fgalea@free.fr>
 */

#ifndef _FRZ_PIXBUF_HPP_
#define _FRZ_PIXBUF_HPP_

#include <cstdlib>
#include <iostream>
#include <stdint.h>

namespace Frz {

class Pixbuf
{
protected:
  int width, height, pitch;
  uint32_t * pixbuf;

public:
  Pixbuf(int w, int h, bool allocPixbuf = false);
  virtual ~Pixbuf();
  void saveBitmap(const char *filename);
};

}; // namespace Frz

#endif
