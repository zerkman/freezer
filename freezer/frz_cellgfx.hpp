/* Cell Broadband Engine subsystem.
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
/*
 * frz_cellgfx.hpp - Cell BE subsystem for the Freezer animation system
 * by François Galea <fgalea@free.fr>
 */

#ifndef _EFFECT_SPE_HPP_
#define _EFFECT_SPE_HPP_

#include <libspe2.h>
#include <pthread.h>
#include "frz_pixbuf.hpp"

typedef int qword __attribute__((vector_size(16)));

#include "spu_engine.h"

#define SPE_COUNT 6

namespace Frz {

class CellGfx : public Pixbuf
{
public:
  CellGfx(int w, int h, bool allocPixbuf = false);
  ~CellGfx();

  struct ThrData {
    pthread_t id;
    spe_context_ptr_t context;
    spe_data data __attribute((aligned(16)));
  };

protected:
  ThrData thr[SPE_COUNT] __attribute((aligned(16)));
  void in_mbox_write(int i, uint32_t* tab, int count);
  int out_mbox_status(int i) {
    return spe_out_mbox_status(thr[i].context);
  }
  uint32_t out_mbox_read(int i) {
    uint32_t ret;
    spe_out_mbox_read(thr[i].context, &ret, 1);
    return ret;
  }
  uint32_t out_intr_mbox_read(int i=0) {
    uint32_t ret;
    spe_out_intr_mbox_read(thr[i].context, &ret, 1, SPE_MBOX_ALL_BLOCKING);
    return ret;
  }

private:
  static void * thread_main(void * arg);
};

}; // namespace Frz

#endif
