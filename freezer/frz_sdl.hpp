/* SDL environment.
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

class SDLSystem : public System {
protected:
  SDL_Surface *screen;
  uint32_t _t0;

  static void vsync();
  static bool test_exit();

public:
  SDLSystem(const char *title, Script &s, int width, int height, int videomode,
            uint32_t t0=0);
  virtual ~SDLSystem() {}

  static int parseCmd(int argc, char **argv, int &width, int &height,
      int &videomode, uint32_t &t0);

  int operator()();
};


}; // namespace Frz



#endif

