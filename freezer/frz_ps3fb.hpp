/* PS3 framebuffer graphics environment.
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

/*! \brief Base class for user input device.
 *
 * This is a simple framework for either handling or simulating a PS3
 * controller.
 */
class Buttons {
public:
  //! Default constructor.
  Buttons();
  virtual ~Buttons() {}

  /*! \brief Returns a bitfield of the currently pressed buttons */
  virtual int getAllButtons() = 0;
  /*! \brief Return the status of button \p b. */
  int getButton(int b);

protected:
  /*! \brief Updates the local copy of button status. */
  virtual void update() = 0;
  /*! \brief Return the status of button \p b without performing a status
   * update. */
  int getButtonNoUpdate(int b);
  //! Internal bitfield containing a copy of the status of all buttons.
  unsigned int buttons[256/sizeof(unsigned int)];
};

/*! \brief PS3 controller input device.
 *
 * This is a Buttons extension for the PS3 controller.
 */
class Joystick: public Buttons {
public:
  //! Default constructor.
  Joystick();
  virtual ~Joystick();
  //! Get the value of selected axis
  int getAxis(int a);
  virtual int getAllButtons();

protected:
  virtual void update();

private:
  int joyFD;
  int axis[4];
};

/*! \brief Keyboard input device.
 *
 * This is a Buttons extension for the Linux keyboard.
 */
class Keyboard: public Buttons {
public:
  //! Default constructor.
  /*! Sets the keyboard mode to raw. */
  Keyboard();
  //! Default destructor.
  /*! Resets the keyboard mode to normal. */
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

//! Interface to the framebuffer.
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
  /*! \brief Constructor.
   *
   * Sets the console to graphics mode, and deactivates the cursor.
   */
  FrameBuffer();
  //! Destructor.
  ~FrameBuffer();

  //! Returns the current off-screen framebuffer.
  void* getFrameBuffer();
  //! Returns the currently visible framebuffer.
  void* getOnScreenFrameBuffer();
  //! Flips to the next off-screen frame.
  void flip();
  //! Wait for vsync.
  void vsync();
  //! Initialize the FPS counter.
  void initFPS();
  //! Returns the FPS rate to date.
  float getFPS();
  //! Returns the framecount to date.
  int getFramecount();
  //! Returns the screen width in pixels.
  uint32_t getWidth() { return width; }
  //! Returns the screen height in pixels.
  uint32_t getHeight() { return height; }
};

/*! \brief PS3 framebuffer graphics environment.
 *
 * This is a graphics environment which uses the PS3 framebuffer for graphics
 * output.
 *
 * The environment adapts itself to the screen resolution.
 *
 * The main loop of the animation system uses a Buttons derived object to test
 * for the user exit of the animation: if the user presses the Esc key (using a
 * Keyboard object) or the circle button (using a Joystick object) the loop
 * exits.
 */
class PS3FBSystem : public System {
  FrameBuffer &fb;
  Buttons &b;
  bool _pause;

public:
  /*! \brief Constructs a SDLSystem object.
   *
   * \param s The animation script object.
   * \param _fb The current frame buffer.
   * \param _b A Keyboard or Joystick object, for main loop exit test.
   */
  PS3FBSystem(Script &s, FrameBuffer &_fb, Buttons &_b);
  virtual ~PS3FBSystem() {}

  /*! \brief Executes the animation.
   *
   * The animation stops when the user presses the Esc key (using a Keyboard
   * object) or the circle button (using a Joystick object).
   */
  int operator()();
};

}; // namespace Frz

#endif


