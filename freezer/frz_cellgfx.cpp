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
 * frz_cellgfx.cpp - Cell BE subsystem for the Freezer animation system
 * by François Galea <fgalea@free.fr>
 */

#include <iostream>

#include "frz_cellgfx.hpp"
#include "frz_misc.hpp"

extern spe_program_handle_t spu_engine_handle;

using namespace Frz;

void * CellGfx::thread_main(void * arg) {
  ThrData *a = (ThrData *)arg;
  int retval;
  unsigned int entry_point = SPE_DEFAULT_ENTRY; 

  retval = spe_context_run(a->context, &entry_point, 0, &a->data, NULL, NULL);
  if(retval) {
    perror("An error occurred running the SPE program");
  }
  return NULL;
}

void CellGfx::in_mbox_write(int i, uint32_t* tab, int count) {
  int ret;
  while (count) {
    ret = spe_in_mbox_write(thr[i].context, tab, count, SPE_MBOX_ALL_BLOCKING);
    if (ret == -1) {
      std::cerr<<"in_mbox_write error"<<std::endl;
      exit(1);
    }
    tab += ret;
    count -= ret;
  }
}

CellGfx::CellGfx(int w, int h, bool allocPixbuf) : Pixbuf(w, h, allocPixbuf)
{
  int i, j;
  uint64_t sig1_ea;
  for (i=0; i<SPE_COUNT; i++) {
    thr[i].context = spe_context_create(
        SPE_EVENTS_ENABLE|SPE_MAP_PS|SPE_CFG_SIGNOTIFY1_OR, NULL);
    spe_program_load(thr[i].context, &spu_engine_handle);
    thr[i].data.spe_rank = i;
    thr[i].data.spe_count = SPE_COUNT;
    thr[i].data.width = width;
    thr[i].data.height = height;
    thr[i].data.pitch = width * 4;
    sig1_ea = spu_ea(spe_ps_area_get(thr[i].context,
        SPE_SIG_NOTIFY_1_AREA)) + 12;
    for (j=0; j<SPE_COUNT; j++)
      thr[j].data.sig1[i] = sig1_ea;
  }

  for (i=0; i<SPE_COUNT; i++) {
    if (pthread_create(&thr[i].id, NULL, thread_main, &thr[i]) != 0) {
      std::cerr << "pthread_create error" << std::endl;
      exit(1);
    }
  }
}

CellGfx::~CellGfx()
{
  int i;
  unsigned int zero = 0;
  /* Wait for SPE threads completion */
  for (i=0; i<SPE_COUNT; i++)
    in_mbox_write(i, &zero, 1);
  for (i=0; i<SPE_COUNT; i++)
    pthread_join(thr[i].id, NULL);
}

