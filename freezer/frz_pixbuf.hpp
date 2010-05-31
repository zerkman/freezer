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

/*! \brief Pixel buffer manager class.
 *
 * This class eventually handles a 32-bit ARGB pixel buffer of required width
 * and height.
 */
class Pixbuf
{
protected:
  //! Surface width in pixels.
  int width;
  //! Surface height in pixels.
  int height;
  //! Surface pitch in pixels.
  int pitch;
  //! Pixel buffer address, if allocated, \c 0 otherwise.
  uint32_t * pixbuf;

public:
  /*! \brief Constructs a Pixel buffer manager object.
   *
   * \param w Picture width in pixels.
   * \param h Picture height in pixels.
   * \param allocPixbuf if \b true, allocate a screen buffer.
   */
  Pixbuf(int w, int h, bool allocPixbuf = false);
  //! Destructor.
  virtual ~Pixbuf();
  /*! \brief Exports the buffer into a bitmap file.
   *
   * The export file format is uncompressed 24-bit BMP.
   *
   * \param filename The name of the file to be created.
   */
  void saveBitmap(const char *filename);
};

}; // namespace Frz

#endif
