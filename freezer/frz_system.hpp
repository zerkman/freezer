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

#ifndef _FRZ_SYSTEM_HPP_
#define _FRZ_SYSTEM_HPP_

#include "frz_cellgfx.hpp"
#include "frz_script.hpp"
#include "frz_misc.hpp"
#include "spu_engine.h"

#define MAX_TM 16

namespace Frz {

struct synchro {
  uint32_t count[32];
};

typedef qword triangle_data[8];

class System : public CellGfx
{
  Script &script;
  spe_3d_data data3d __attribute((aligned(16)));
  spe_frame_data fd __attribute((aligned(16)));
  alignvec<triangle_data> td[LIST_COUNT];
  alignvec<synchro> sync;
  bool first_draw;

protected:
  const char *ts[MAX_TM];
  uint32_t t[MAX_TM];
  trans transforms[MAX_OBJ];
  uint32_t uTime();

public:
  System(Script &s, int w, int h, bool allocPixbuf = false);
  virtual ~System();

  virtual void draw(uint32_t time, void * buf = 0);

  virtual int operator()() = 0;
};

}; // namespace Frz


#endif
