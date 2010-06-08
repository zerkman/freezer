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

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <sys/time.h>

#include "frz_system.hpp"
#include "frz_obj3d.hpp"

using namespace Frz;

System::System(Script &s, int w, int h, bool allocPixbuf) :
    CellGfx(w, h, allocPixbuf), script(s), sync(128, 1), first_draw(true) {
  int i;
  uint64_t ea;
  memset(&t[0], 0, sizeof(t));
  ts[0] = "total";
  ts[1] = "System::draw";
  ts[2] = "rtpr";
  ts[3] = "setupFrame";
  ts[4] = "draw_screen";

  for (i=0; i<LIST_COUNT; i++)
    td[i].setFixedCapacity(262144*4/LIST_COUNT);

  fd.oldscreen_ea = 0;
  ea = spu_ea(&data3d);

  /* Send address of data3d structure */
  for (i = 0; i < SPE_COUNT; i++)
    in_mbox_write(i, (uint32_t*)(void*)&ea, 2);

  /* Wait for spu's to be ready */
  for (i = 0; i < SPE_COUNT; i++) {
    while (out_mbox_status(i) == 0);
    out_mbox_read(i);
  }
  t[0] = uTime();
}

System::~System() {
  int i;
  t[0] = uTime() - t[0];
  for (i=0; i<MAX_TM; i++) {
    if (t[i])
      std::cout << std::setw(20) << std::left << ts[i] << " "
          << std::setw(10) << std::right << t[i]
          << " " << float(t[i])/t[0] << std::endl;
  }
}

uint32_t System::uTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec*1000000 + tv.tv_usec;
}

void System::draw(uint32_t time, void * buf) {
  int i;
  uint64_t ea;
  uint32_t msg;
  uint32_t t0 = uTime();
  uint32_t t1, t2, t3, t4;

  memset(&sync[0], 0, 128);

  if (buf == 0)
    buf = pixbuf;
  ea = spu_ea(buf);
  if (ea & 0x0F) {
    std::cerr << "invalid screen address: " << buf << std::endl;
    exit(1);
  }
  fd.screen_ea = ea;
  fd.backgdc = script.getBackgroundColor();
  fd.blur_strength = 0.50001f;
  fd.blur_strength1 = 0.49999f;
  fd.filter_color = script.getFadeColor();
  fd.filter_strength = script.getFadeStrength();
  if (script.getFlash())
    fd.flash_strength = 255.f;
  else
    fd.flash_strength *= 0.5f;

  if (first_draw) {
    script.setupFrame(time, transforms);
    for (i = 0; i < script.getObjectCount(); i++)
      make_rotation(fd.rot[i], transforms[i]);
    first_draw = false;
  }

  data3d.e3d_ea = spu_ea(this);
  data3d.t_ea = spu_ea(script.getTriangles());
  //data3d.sh_t_ea = spu_ea(script.getSharedTriangles());
  data3d.fd_ea = spu_ea(&fd);
  data3d.sync_ea = spu_ea(&sync[0]);
  for (i=0; i<LIST_COUNT; i++) {
    data3d.td_ea[i] = spu_ea(&td[i][0]);
  }
  data3d.n_obj = script.getObjectCount();
  data3d.n_t = script.getTriangleCount();
  data3d.time = time;
  //data3d.n_shared = script.getSharedTriangleCount();
  data3d.lightsource = script.getLightSource().v();

  t1 = uTime();
  /* Send new frame message */
  msg = 1;
  for (i = 0; i < SPE_COUNT; i++)
    in_mbox_write(i, &msg, 1);

  /* Wait for SPEs to have generated 2D triangles */
  while (out_mbox_status(0) == 0)
    pthread_yield();
  out_mbox_read(0);
  t2 = uTime();
  t[2] += t2-t1;

  /* Prepare next frame as the SPEs are rendering */
  script.setupFrame(time, transforms);
  for (i = 0; i < script.getObjectCount(); i++)
    make_rotation(fd.rot[i], transforms[i]);
  t3 = uTime();
  t[3] += t3-t2;

  /* Wait for SPEs to finish screen drawing */
  while (out_mbox_status(0) == 0)
    pthread_yield();
  out_mbox_read(0);
  t4 = uTime();
  t[4] += t4-t2;
  t[1] += t4-t0;
  fd.oldscreen_ea = ea;
}

