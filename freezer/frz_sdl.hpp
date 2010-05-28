/* SDL graphics environment.
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

#ifndef _FRZ_SDL_HPP_
#define _FRZ_SDL_HPP_

#include "frz_system.hpp"
#include "SDL.h"


namespace Frz {

/*! \brief SDL graphics environment.
 *
 * This is a graphics environment which uses the SDL library for graphics
 * output.
 */
class SDLSystem : public System {
  //! Allocated SDL screen buffer.
  SDL_Surface *screen;
  //! Start time of animation.
  uint32_t _t0;

  //! Simulates vertical synchronization delay.
  static void vsync();
  //! Tests for exit event (window close).
  static bool test_exit();

public:
  /*! \brief Constructs a SDLSystem object.
   *
   * \param title Window title.
   * \param s The animation script object.
   * \param width Picture width in pixels.
   * \param height Picture height in pixels.
   * \param videomode SDL video mode.
   * \param t0 Start time value in milliseconds.
   */
  SDLSystem(const char *title, Script &s, int width, int height, int videomode,
            uint32_t t0=0);
  virtual ~SDLSystem() {}

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
   * \param videomode SDL video mode.
   * \param t0 Start time value in milliseconds.
   */
  static int parseCmd(int argc, char **argv, int &width, int &height,
      int &videomode, uint32_t &t0);

  /*! \brief Executes the animation. */
  int operator()();
};


}; // namespace Frz



#endif

