/* Example program: a pair of differently-shaded toruses.
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

class Torus : public Frz::Object3d {
public:
  Torus(float r1, float r2, int n1, int n2, uint32_t color) {
    float a1, a2;
    float sin1, cos1, sin2, cos2;
    float x, y, z, x2, x1, y1;
    int i, j, k;
    int a, b, c, d;

    for (i=0; i<n1; i++)
    {
      a1 = i*M_PI*2/n1;
      sin1 = sinf(a1);
      cos1 = cosf(a1);
      x1 = cos1 * r1;
      y1 = sin1 * r1;
      for (j=0; j<n2; j++)
      {
        a2 = j*M_PI*2/n2;
        sin2 = sinf(a2);
        cos2 = cosf(a2);
        x2 = r2 * cos2;
        x = x2 * cos1 + x1;
        y = x2 * sin1 + y1;
        z = r2 * sin2;
        addVertex(x, y, z);
      }
    }

    k = 0;
    for (i=0; i<n1; i++)
    {
      for (j=0; j<n2; j++)
      {
        a = k + j;
        b = k + (j+1)%n2;
        c = (k + (j+1)%n2 + n2)%(n2*n1);
        d = (k + j + n2)%(n2*n1);
        addPoly(a, b, c, d, color);
      }
      k += n2;
    }
  }
};

class ScnTorus : public Frz::Scene {
protected:

public:
  ScnTorus() {
    Torus obj1(150.0f, 50.0f, 60, 35, 0xffe080);

    add_object(obj1, SH_FLAT);
    add_object(obj1, SH_RGBSMOOTH);
  }

  virtual ~ScnTorus() {};

  virtual void setupFrame(uint32_t time, Frz::trans transforms[]) {
    unsigned int i;
    int div = 1000;

    for (i=0; i<2; i++) {
      transforms[i].a = (float)time / div;   /* rotation angle according to x */
      transforms[i].b = (float)time / div;   /* rotation angle according to y */
      transforms[i].c = (float)time / div;   /* rotation angle according to z */
      transforms[i].x = sinf(((float)time)/1000 + i*M_PI)*150.0f;
      transforms[i].y = 0.0f;
      transforms[i].z = -400.0f;

      div += 200;
    }
  }
};

class MyScript: public Frz::Script {
  Frz::Scene *scene;
public:
  MyScript() {
    scene = new ScnTorus();
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
  Frz::SDLSystem s("torus demo", script, width, height, videomode, t0);
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
