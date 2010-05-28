/* Bitmap file generation environment.
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

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "frz_bmp.hpp"

/* framerate in fps */
#define FRAMERATE 60

#define WIDTH 256
#define HEIGHT 144
#define MIN 0

using namespace Frz;

int BMPSystem::parseCmd(int argc, char **argv, int &width, int &height,
    int &min, int &max, int &step, bool &gen) {
  int i;
  width = WIDTH;
  height = HEIGHT;
  min = MIN;
  max = MIN+1;
  step = 4;
  gen = false;

  for (i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-w") && (i+1)<argc) {
      i++;
      width = atoi(argv[i]);
    }
    else if (!strcmp(argv[i], "-h") && (i+1)<argc) {
      i++;
      height = atoi(argv[i]);
    }
    else if (!strcmp(argv[i], "-s") && (i+1)<argc) {
      i++;
      step = atoi(argv[i]);
    }
    else if (!strcmp(argv[i], "-g")) {
      gen = true;
    }
    else if (i == argc-2 && isdigit(argv[i][0]) && isdigit(argv[i+1][0])) {
      min = atoi(argv[i++]);
      max = atoi(argv[i]);
    }
    else if (i == argc-1 && isdigit(argv[i][0])) {
      min = atoi(argv[i]);
      max = min + 1;
    }
    else {
      std::cerr<<"usage: "<<argv[0]<<" [OPTION]... [MIN] [MAX]"<<std::endl;
      std::cerr<<"Generate frames between number MIN (default="<<MIN
          <<") and MAX (default=MIN+1)"<<std::endl;
      std::cerr<<"  -w width\tframe width (default="<<WIDTH<<")"<<std::endl;
      std::cerr<<"  -h height\tframe height (default="<<HEIGHT<<")"<<std::endl;
      std::cerr<<"  -s step\tstep between frame numbers (default=4)"<<std::endl;
      std::cerr<<"  -g\t\tgenerate picture file(s)"<<std::endl;
      return 1;
    }
  }
  return 0;
}

BMPSystem::BMPSystem(Script &s, int width, int height, int min, int max,
    int step, bool gen): System(s, width, height, false), _min(min),
    _max(max), _step(step), _gen(gen) {
}

BMPSystem::~BMPSystem() {
}

int BMPSystem::operator()() {
  int i;
  int pic_num = 0;
  void * tmpbuf;
  uint32_t * buf;
  if (posix_memalign(&tmpbuf, 128, pitch*height*2+128)) {
    std::cerr<<"Insufficient memory"<<std::endl;
    exit(1);
  }
  buf = (uint32_t*)tmpbuf;
  for (i=_min; i<_max; i+=_step)
  {
    pixbuf = buf + ((pic_num*pitch*height+127)&-128)/4;
    draw(i);
    if (_gen) {
      std::ostringstream ss;
      ss << "pic_" << std::setw(6) << std::setfill('0') << i << ".bmp";
      saveBitmap(ss.str().c_str());
    }
    pic_num = 1-pic_num;
  }
  pixbuf = 0;
  free(buf);
  return 0;
}

