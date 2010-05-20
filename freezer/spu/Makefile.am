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

c_sources = spu_engine.c
h_sources = ../cos_sin18_v.h ../frz_defs.h ../spu_engine.h

CC=$(SPUCC)
AM_CFLAGS = -I..

noinst_PROGRAMS = spu_engine

spu_engine_SOURCES = $(c_sources) $(h_sources)
