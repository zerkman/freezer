/* Bitmap file generation graphics environment.
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

#ifndef _FRZ_BMP_HPP_
#define _FRZ_BMP_HPP_

#include "frz_system.hpp"

namespace Frz {

/*! \brief Bitmap file generation graphics environment
 *
 * This is a graphics environment which generates one frame or a set of
 * frames in a time interval, and eventually outputs them into one or several
 * bitmap files.
 *
 * The bitmap files are in 24-bit BMP format, and are named
 * <tt>pic_xxxx.bmp</tt> where \c xxxx is the time value corresponding to the
 * frame. 
 */
class BMPSystem : public System {
  int _min, _max, _step;
  bool _gen;

public:
  /*! \brief Constructs a BMPSystem object.
   *
   * \param s The animation script object.
   * \param width Picture width in pixels.
   * \param height Picture height in pixels.
   * \param min Minimum time value.
   * \param max Maximum time value.
   * \param step Time step between consecutive pictures.
   * \param gen If \b true, generate bitmap files.
  */
  BMPSystem(Script &s, int width, int height, int min, int max, int step,
    bool gen);
  virtual ~BMPSystem();

  /*! \brief Parses the command line and populate initialization data.
   *
   * The command line is parsed for specific values of the constructor
   * arguments. If some values are not provided, default values are set.
   * All arguments passed by reference are modified.
   *
   * \param argc First argument of the \c main function.
   * \param argv Second argument of the \c main function.
   * \param width Picture width in pixels.
   * \param height Picture height in pixels.
   * \param min Minimum time value.
   * \param max Maximum time value.
   * \param step Time step between consecutive pictures.
   * \param gen If \b true, generate bitmap files.
   */
  static int parseCmd(int argc, char **argv, int &width, int &height,
    int &min, int &max, int &step, bool &gen);

  /*! \brief Executes the animation. */
  virtual int operator()();
};


}; // namespace Frz



#endif

