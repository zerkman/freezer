/* Animation system.
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

/*! \mainpage Freezer
    \author Fran&ccedil;ois Galea
    \version $(VERSION)

\section MainIntro Introduction

Freezer is a 3D engine and animation system for the Cell Broadband Engine
processor.

Freezer is distributed under the terms of the GNU Lesser General Public
Licence.

The 3D engine main characteristics are:

- Efficient use of SPE's for display acceleration. Used PPE CPU power is reduced
to the minimum (almost zero).
- Support for triangles, quads, with optional extrusion mechanism for 2D ojects.
- Support for import of 3D objects in OFF file format.
- Available shading types:
  - flat shading
  - gouraud shading
- Accurate Z-buffered rendering.
- Automatic generation of normal vectors for flat-shaded objects, and for
Gouraud-shaded objects.
- No support for textures (yet).

The animation system characteristics are:

- Available output environments: SDL, PS3 framebuffer, or file export for frames
(BMP format).
- Double buffering.
- Eventual support for motion blur and fading effects.

\section MainTOC Table Of Contents

- \ref InstMan "Installation of the library". This page indicates the
  procedure to generate the library from source code.
- \ref CompMan - Compile and execute a sample program.
- \ref QuickTutorial - A step-by-step tutorial about what you can do using
  Freezer.

\section Links Links

- <a href="http://freezer.sector1.fr/">The official Freezer website</a>,
where you can find the latest version of this documentation.
- <a href="http://github.com/fgalea/freezer/">The Freezer project page on
Github</a>, allowing to browse the source code and examine the different
commits.


\page InstMan Freezer library installation

This page concerns the setup and installation of Freezer, either on a native
Cell BE host, or on a cross-compilation setup.

\par Required tools
Freezer requires the following tools to be installed to be able to set up and
build the library:
  - git
  - autoconf
  - automake
  - libtool
  - make
  - g++ for ppu
  - gcc for spu (spu-gcc)
  - eventually SDL.

\par Source code download
To download the latest repository version, issue the commands:
\verbatim
$ git clone http://github.com/fgalea/freezer.git
$ cd freezer
\endverbatim
then, create the autotools script files:
\verbatim
$ ./bootstrap
\endverbatim

\par Source code setup
If using on a Cell BE host, just execute the \c configure script:
\verbatim
$ ./configure
\endverbatim
If cross compiling, you must provide a host type as argument of the \c
configure script, either:
 - \verbatim $ ./configure --host=ppu \endverbatim for 64-bit PPU code
   generation, or
 - \verbatim $ ./configure --host=ppu32 \endverbatim for 32-bit PPU code.

\par
If any of the previous command fails, this is certainly because one of the
required tools listed above is missing.

\par Building the library
If the setup process went ok, you should be able to build the library using
the command:
\verbatim
$ make
\endverbatim

\par Installation of the library
The default installation directory is <tt>/usr/local</tt>. You then need the
root privileges to be able to install the library. The following command will
do the job:
\verbatim
$ make install
\endverbatim

\par
If you do not have the root privileges, you still can install the library to
a custom directory. For this, during the setup phase you need to specify the
installation directory using the \c --target argument to the \c configure
script, like for instance:
\verbatim
$ ./configure --prefix=path/to/installation/directory
\endverbatim

\par Compilation and execution of the example programs
After having compiled the library, even without installing, you can try and
compile the example programs using the commands:
\verbatim
$ cd examples
$ make
\endverbatim
\par
You should end up with a bunch of various executable files, each demo program
coming in three variants, with \c _sdl, \c _ps3fb and \c _bmp suffixes. The
SDL version is preferred when running in windowed mode, the ps3fb version is
for runnnig from console mode, and the bmp version is just a test program to
generate the images in bmp-formatted files, not really useful unless when
debugging (or using the cell simulator when away from your ps3 for instance).

\page CompMan Use Freezer in your own programs

\todo complete the page.

\page QuickTutorial Freezer Quick Tutorial

\todo complete the page.

*/


#ifndef _FRZ_SYSTEM_HPP_
#define _FRZ_SYSTEM_HPP_

#include "frz_cellgfx.hpp"
#include "frz_script.hpp"
#include "frz_misc.hpp"
#include "spu_engine.h"

#define MAX_TM 16

namespace Frz {

typedef qword triangle_data[8];

/*! \brief Animation system virtual class */
class System : public CellGfx
{
  struct synchro {
    uint32_t count[32];
  };

  Script &script;
  spe_3d_data data3d __attribute((aligned(16)));
  spe_frame_data fd __attribute((aligned(16)));
  alignvec<triangle_data> td[LIST_COUNT];
  alignvec<synchro> sync;
  bool first_draw;

protected:
  //! Time counter names.
  const char *ts[MAX_TM];
  //! Time counter values.
  uint32_t t[MAX_TM];
  //! The array of transforms.
  trans transforms[MAX_OBJ];
  //! Returns the current time. Used for calculating the time counter values.
  uint32_t uTime();

public:
  /*! \brief Constructs an animation system object.
   *
   * \param s The animation script object.
   * \param w Picture width in pixels.
   * \param h Picture height in pixels.
   * \param allocPixbuf if \b true, allocate a screen buffer.
   */
  System(Script &s, int w, int h, bool allocPixbuf = false);
  virtual ~System();

  /*! \brief Draws a new frame.
   *
   * \param time Current animation time in milliseconds.
   * \param buf If not zero, used as screen buffer address. Otherwise, use
   *            the screen address allocated by the constructor.
   */
  virtual void draw(uint32_t time, void * buf = 0);

  /*! \brief Executes the animation.
   *
   * This virtual operator must be implemented by the graphics environment.
   */
  virtual int operator()() = 0;
};

}; // namespace Frz


#endif
