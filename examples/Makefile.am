# Data file for the automake makefile generator
# Copyright (C) 2010 Francois Galea <fgalea@free.fr>
#
# This file is part of Freezer.
#
# Freezer is free software: you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# Freezer is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
# License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Freezer.  If not, see <http://www.gnu.org/licenses/>.

BMP_PROG = cube_bmp cube2_bmp torus_bmp teapot_bmp

if USE_PS3FB
  PS3FB_PROG = cube_ps3fb cube2_ps3fb torus_ps3fb teapot_ps3fb
endif

if USE_SDL
  SDL_PROG = cube_sdl cube2_sdl torus_sdl teapot_sdl
endif

AM_CXXFLAGS = $(PPU_CFLAGS) $(INCSDL) $(INCCELL)
AM_LDFLAGS = -L../freezer -lfreezer $(LIBSDL) $(LIBCELL)

EXTRA_DIST = teapot.off

noinst_PROGRAMS = $(BMP_PROG) $(PS3FB_PROG) $(SDL_PROG)

cube_bmp_SOURCES = cube.cpp
cube_bmp_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_BMP
cube_ps3fb_SOURCES = cube.cpp
cube_ps3fb_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_PS3FB
cube_sdl_SOURCES = cube.cpp
cube_sdl_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_SDL

cube2_bmp_SOURCES = cube2.cpp
cube2_bmp_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_BMP
cube2_ps3fb_SOURCES = cube2.cpp
cube2_ps3fb_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_PS3FB
cube2_sdl_SOURCES = cube2.cpp
cube2_sdl_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_SDL

torus_bmp_SOURCES = torus.cpp
torus_bmp_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_BMP
torus_ps3fb_SOURCES = torus.cpp
torus_ps3fb_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_PS3FB
torus_sdl_SOURCES = torus.cpp
torus_sdl_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_SDL

teapot_bmp_SOURCES = teapot.cpp
teapot_bmp_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_BMP
teapot_ps3fb_SOURCES = teapot.cpp
teapot_ps3fb_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_PS3FB
teapot_sdl_SOURCES = teapot.cpp
teapot_sdl_CXXFLAGS = $(AM_CXXFLAGS) -DFRZ_SDL
