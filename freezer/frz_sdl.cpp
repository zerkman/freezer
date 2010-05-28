/* SDL environment.
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
#include <cstdlib>

#include "frz_sdl.hpp"

/* framerate in fps */
#define FRAMERATE 60

#define WIDTH 768
#define HEIGHT 432

using namespace Frz;

static Uint8 pause_=0;

int SDLSystem::parseCmd(int argc, char **argv, int &w, int &h, int &vm,
    uint32_t &t0) {
  vm = SDL_DOUBLEBUF | SDL_HWSURFACE;
  //videomode = SDL_SWSURFACE;
  w = WIDTH;
  h = HEIGHT;
  bool has_t0 = false;
  int i;
  t0 = 0;

  for( i = 1; i < argc; i++ ) {
    if( !strcmp( argv[i], "-fs" ) )
      vm |= SDL_FULLSCREEN;
    else if( !strcmp( argv[i], "-w" ) && (i+1)<argc ) {
      i++;
      w = atoi( argv[i] );
    }
    else if( !strcmp( argv[i], "-h" ) && (i+1)<argc ) {
      i++;
      h = atoi( argv[i] );
    }
    else if (!has_t0) {
      has_t0 = true;
      t0 = atoi(argv[i]);
    }
    else {
      std::cerr<<"usage: "<<argv[0]<<" [-fs] [-w <width>] [-h <height>] [time0]"
          <<std::endl;
      return 1;
    }
  }
  return 0;
}

void SDLSystem::vsync() {
  static Uint32 next_time = 0;
  Uint32 now = SDL_GetTicks();
  if( now < next_time )
    SDL_Delay( next_time-now );
  next_time += 1000 / FRAMERATE;
}

int SDLSystem::test_exit( ) {
  SDL_Event event;
  SDLKey sym;
  static int esc_pressed = 0;

  int ret = 0;
  while( SDL_PollEvent( &event ) )
    switch( event.type )
  {
    case SDL_KEYDOWN:
      sym = event.key.keysym.sym;
      esc_pressed = (sym == SDLK_ESCAPE);
      pause_ |= (sym == SDLK_SPACE);
      /* printf( "key pressed = %d\n", sym ); */
      break;
    case SDL_KEYUP:
      sym = event.key.keysym.sym;
      if( (sym == SDLK_ESCAPE) && esc_pressed )
	ret = 1;
      pause_ &= !(sym == SDLK_SPACE);
      break;
    case SDL_QUIT:
      ret = 1;
  }
  return ret;
}

SDLSystem::SDLSystem(const char *title, Script &s, int width, int height,
                     int videomode, uint32_t _t0)
    : System(s, width, height, false), screen(0), t0(_t0) {
  /* Initialize the SDL library */
  if( SDL_Init(SDL_INIT_VIDEO) < 0 )
  {
    std::cerr<<"Couldn't initialize SDL: "<<SDL_GetError()<<std::endl;
    exit(1);
  }

  /* Clean up on exit */
  atexit(SDL_Quit);

  /*
   * Initialize the display in a width*height 32-bit mode,
   * requesting a hardware surface
   */
  screen = SDL_SetVideoMode(width, height, 32, videomode);
  if ( screen == NULL ) {
    std::cerr<<"Couldn't set video mode: "<<SDL_GetError()<<std::endl;
    exit(1);
  }
  SDL_WM_SetCaption(title,title);
}

int SDLSystem::operator()() {
  uint32_t tm;
  int pic_num = 0;
  ts[8] = "SDL_Flip";
  void * tmpbuf;
  if (posix_memalign(&tmpbuf, 128, pitch*height*2+128)) {
    std::cerr<<"Insufficient memory"<<std::endl;
    exit(1);
  }
  uint32_t * buf = (uint32_t*)tmpbuf;
  while (!test_exit())
  {
    if ( SDL_MUSTLOCK(screen)) {
      if (SDL_LockSurface(screen) < 0) {
        std::cerr<<"Can't lock screen: "<<SDL_GetError()<<std::endl;
        return 1;
      }
    }
    pixbuf = buf + ((pic_num*pitch*height+127)&-128)/4;

    if (((uint64_t)screen->pixels) & 0x0FLL) {
      std::cerr<<"Unaligned screen buffer: "<<screen->pixels<<std::endl;
      return 1;
    }
    if (!pause_) {
      draw(SDL_GetTicks()+t0);
      memcpy(screen->pixels, pixbuf, pitch*height);
    }

    if (SDL_MUSTLOCK(screen)) {
      SDL_UnlockSurface(screen);
    }
    vsync();
    tm = uTime();
    SDL_Flip(screen);
    //t[8] += uTime()-tm;
    pic_num = 1-pic_num;
  }
  pixbuf = 0;
  return 0;
}

