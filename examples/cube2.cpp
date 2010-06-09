/* Example program: a flat shaded cube obtained by extruding a quad.
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

#include "freezer/frz_script.hpp"

#ifdef FRZ_SDL
#include "freezer/frz_sdl.hpp"
#elif defined(FRZ_BMP)
#include "freezer/frz_bmp.hpp"
#elif defined(FRZ_PS3FB)
#include "freezer/frz_ps3fb.hpp"
#endif


class ScnCube : public Frz::Scene {
protected:

public:
  ScnCube(uint32_t color=0xffffff) {
    int16_t r = 100;

    Frz::vertex a(-r, -r, 0);
    Frz::vertex b(-r, +r, 0);
    Frz::vertex c(+r, +r, 0);
    Frz::vertex d(+r, -r, 0);
    add_quad(SH_FLAT, 0, 0, 0xffffff, a, b, c, d);
    getTriangles()[0].setThickness(r*2);
    add_object();
  }

  virtual ~ScnCube() {};

  virtual void setupFrame(uint32_t time, Frz::trans transforms[]) {
    unsigned int i;
    int div = 1000;

    transforms[i].a = (float)time / div;   /* rotation angle according to x */
    transforms[i].b = (float)time / div;   /* rotation angle according to y */
    transforms[i].c = (float)time / div;   /* rotation angle according to z */
    //transforms[i].x =-100.0f + 200.0f*i;
    transforms[i].x = 0.0f;
    transforms[i].y = 0.0f;
    transforms[i].z = -400.0f;
  }
};

class MyScript: public Frz::Script {
  Frz::Scene *scene;
public:
  MyScript() {
    scene = new ScnCube();
    setScene(scene);
  }
  virtual ~MyScript() {
    delete scene;
  }
  virtual void setupFrame(uint32_t time) {
    // Do nothing.
  }
};

int main(int argc, char **argv) {
#ifdef FRZ_SDL
  int width, height, videomode;
  uint32_t t0;
  if (Frz::SDLSystem::parseCmd(argc, argv, width, height, videomode, t0))
    return 1;
  MyScript script;
  Frz::SDLSystem s("cube demo", script, width, height, videomode, t0);
#elif defined(FRZ_BMP)
  int width, height, min, max, step;
  bool gen;
  if (Frz::BMPSystem::parseCmd(argc, argv, width, height, min, max, step, gen))
    return 1;
  MyScript script;
  Frz::BMPSystem s(script, width, height, min, max, step, gen);
#elif defined(FRZ_PS3FB)
  MyScript script;
  //Frz::Joystick j;
  Frz::Keyboard j;
  Frz::FrameBuffer fb;
  Frz::PS3FBSystem s(script, fb, j);
#endif

  s();
  return 0;
}
