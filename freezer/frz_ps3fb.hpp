/* PS3 framebuffer environment.
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

#ifndef _FRZ_PS3FB_HPP_
#define _FRZ_PS3FB_HPP_

#include "frz_system.hpp"
#include <stdint.h>
#include <sys/time.h>
#include <termios.h>

namespace Frz {

enum JoystickStatus {
  SELECT = 0,
  L3,
  R3,
  START,
  UP,
  RIGHT,
  DOWN,
  LEFT,
  L2,
  R2,
  L1,
  R1,
  TRIANGLE,
  CIRCLE,
  CROSS,
  SQUARE,
  PS
};

class Buttons {
public:
  Buttons();
  virtual ~Buttons() {}

  virtual int getAllButtons() = 0;
  int getButton(int b);

protected:
  virtual void update() = 0;
  int getButtonNoUpdate(int b);

  unsigned int buttons[256/sizeof(unsigned int)];
};

class Joystick: public Buttons {
public:
  Joystick();
  virtual ~Joystick();
  int getAxis(int a);
  virtual int getAllButtons();

protected:
  virtual void update();

private:
  int joyFD;
  int axis[4];
};

class Keyboard: public Buttons {
public:
  Keyboard();
  virtual ~Keyboard();
  virtual int getAllButtons();

protected:
  virtual void update();

private:
  int fd;
  int e0;
  int oldkbmode;
  struct termios oldtcattr;
};

class FrameBuffer {
  uint32_t width;             // width of the screen
  uint32_t height;            // height of the screen

  uint32_t left;              // offset into the viewpoint when not fullscreen
  uint32_t top;               // offset into the viewpoint when not fullscreen

  uint32_t right;             // end of the viewpoint when not fullscreen
  uint32_t bottom;            // end of the viewpoint when not fullscreen

  void switchMode(bool gfxMode);
  int consoleFD;

  void* addr;
  void* addr2;
  void* current;
  int length;
  bool frame;

  struct timeval tv;
  int framecount;

public:
  FrameBuffer();
  ~FrameBuffer();

  void* getFrameBuffer();     // gets the current off-screen framebuffer
  void* getOnScreenFrameBuffer();  // gets the currently visible framebuffer
  void flip();                // flips to the next off-screen frame
  void vsync();               // wait for vsync
  void initFPS();             // init FPS counter
  float getFPS();             // get FPS rate to date
  int getFramecount();        // get framecount to date
  uint32_t getWidth() { return width; }
  uint32_t getHeight() { return height; }
};

class PS3FBSystem : public System {
  FrameBuffer &fb;
  Buttons &b;
  bool _pause;

public:
  PS3FBSystem(Script &s, FrameBuffer &_fb, Buttons &_b);
  virtual ~PS3FBSystem() {}

  int operator()();
};

}; // namespace Frz

#endif


