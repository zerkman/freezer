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

SUBDIRS = spu

c_sources = \
 frz_bmp.cpp \
 frz_cellgfx.cpp \
 frz_misc.cpp \
 frz_obj3d.cpp \
 frz_pixbuf.cpp \
 frz_scene.cpp \
 frz_script.cpp \
 frz_system.cpp \
 frz_vertex.cpp

h_sources = \
 spu_engine.h \
 frz_defs.h \
 frz_bmp.hpp \
 frz_cellgfx.hpp \
 frz_misc.hpp \
 frz_obj3d.hpp \
 frz_pixbuf.hpp \
 frz_scene.hpp \
 frz_script.hpp \
 frz_system.hpp \
 frz_vertex.hpp

if USE_SDL
  sdl_sources=frz_sdl.cpp frz_sdl.hpp
endif

if USE_PS3FB
  ps3fb_sources=frz_ps3fb.cpp frz_ps3fb.hpp
endif

library_includedir=$(includedir)/freezer
library_include_HEADERS = $(h_sources)

AM_CPPFLAGS=$(INCSDL) $(INCCELL)
AM_CXXFLAGS=$(PPU_CFLAGS)
AM_LDFLAGS=$(LIBSDL) $(LIBCELL)

lib_LTLIBRARIES = libfreezer.la
libfreezer_la_SOURCES = $(c_sources) $(h_sources) \
  $(sdl_sources) $(ps3fb_sources) spu_engine_csf.lo
libfreezer_la_LIBADD = spu_engine_csf.lo

spu_engine_csf.lo: spu/spu_engine
	../scripts/gcell-embedspu-libtool -c $(EMBEDSPU) spu/spu_engine spu_engine_csf.lo
