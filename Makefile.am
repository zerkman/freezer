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

ACLOCAL_AMFLAGS= -I m4

## top directory
include aminclude.am

#Build in these directories:
SUBDIRS= freezer

#Distribute these directories:
DIST_SUBDIRS = freezer examples scripts

EXTRA_DIST = bootstrap doxygen.cfg

library_includedir=$(includedir)/freezer
library_include_HEADERS=config.h
