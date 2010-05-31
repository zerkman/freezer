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

\ref InstMan "Installation of the library". This
page indicates the procedure to generate the library from source

\section Links Links

- <a href="http://github.com/fgalea/freezer/">The Freezer project page on
Github</a>


\page InstMan Freezer library installation

\todo Complete the page.


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
