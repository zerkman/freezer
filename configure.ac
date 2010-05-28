#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
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

AC_PREREQ([2.65])
AC_INIT([freezer], [0.1.0], [fgalea@free.fr])
AC_CONFIG_SRCDIR([freezer])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

dnl -----------------------------------------------
dnl Package name and version number (user defined)
dnl -----------------------------------------------

GENERIC_LIBRARY_NAME=freezer

#release versioning
GENERIC_MAJOR_VERSION=0
GENERIC_MINOR_VERSION=1
GENERIC_MICRO_VERSION=0

#API version (often = GENERIC_MAJOR_VERSION.GENERIC_MINOR_VERSION)
GENERIC_API_VERSION=0.1
AC_SUBST(GENERIC_API_VERSION)

#shared library versioning
GENERIC_LIBRARY_VERSION=1:0:0
#                       | | |
#                +------+ | +---+
#                |        |     |
#             current:revision:age
#                |        |     |
#                |        |     +- increment if interfaces have been added
#                |        |        set to zero if interfaces have been removed
#                                  or changed
#                |        +- increment if source code has changed
#                |           set to zero if current is incremented
#                +- increment if interfaces have been added, removed or changed


dnl --------------------------------
dnl Package name and version number
dnl --------------------------------

dnl -------- the core of the library
AC_SUBST(GENERIC_LIBRARY_VERSION)

PACKAGE=$GENERIC_LIBRARY_NAME
AC_SUBST(GENERIC_LIBRARY_NAME)

GENERIC_VERSION=$GENERIC_MAJOR_VERSION.$GENERIC_MINOR_VERSION.$GENERIC_MICRO_VERSION
GENERIC_RELEASE=$GENERIC_MAJOR_VERSION.$GENERIC_MINOR_VERSION
AC_SUBST(GENERIC_RELEASE)
AC_SUBST(GENERIC_VERSION)

VERSION=$GENERIC_VERSION

AC_PROG_LIBTOOL

AM_INIT_AUTOMAKE($PACKAGE, $VERSION, no-define)

dnl -------- configure options

AC_ARG_ENABLE(debug,
AC_HELP_STRING([--enable-dCFebug],[use it to obtain the debugging informations in Bob++]),
[enable_debug=$enableval],
[enable_debug=no]
)

AC_ARG_WITH([sdl],
  [AS_HELP_STRING([--with-sdl@<:@=ARG@:>@],
    [Use SDL (no|yes|check) @<:@default=check@:>@])],
  [],
  [with_sdl=check])

AC_ARG_WITH([ps3fb],
  [AS_HELP_STRING([--with-ps3fb@<:@=ARG@:>@],
    [Use PS3FB (no|yes|check) @<:@default=check@:>@])],
  [],
  [with_ps3fb=check])

dnl -------- the doxygen configuration
DX_INIT_DOXYGEN($GENERIC_LIBRARY_NAME, "doxygen.cfg")
DX_HTML_FEATURE(ON)
DX_CHM_FEATURE(OFF)
DX_CHI_FEATURE(OFF)
DX_MAN_FEATURE(OFF)
DX_RTF_FEATURE(OFF)
DX_XML_FEATURE(OFF)
DX_PDF_FEATURE(OFF)
DX_PS_FEATURE(OFF)

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# Checks for compiler switches.
my_save_cflags="$CFLAGS"
CFLAGS=-maltivec
AC_MSG_CHECKING([whether $CC supports -maltivec])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([])],
    [AC_MSG_RESULT([yes])]
    [PPU_CFLAGS=-maltivec],
    [AC_MSG_RESULT([no])
     AC_MSG_FAILURE(The $CC compiler must support the -maltivec flag.)])
CFLAGS="$my_save_cflags"
AC_SUBST([PPU_CFLAGS])

# Checks for libraries.

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h string.h sys/time.h termios.h unistd.h asm/ps3fb.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_PID_T
AC_TYPE_SIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_FORK
AC_FUNC_MMAP
AC_FUNC_STRTOD
AC_CHECK_FUNCS([atexit gettimeofday memset munmap])

# Checks for a Cell Broadband Engine programming environment
SPUCC=
SPUCXX=
EMBEDSPU=
if test "$host_cpu" != powerpc -a "$host_cpu" != powerpc64; then
  with_cell=no
fi
if test "x$with_cell" = x; then
  AC_CHECK_PROGS(SPUCC,spu-gcc)
  if test "x$SPUCC" != "x"; then
    AC_CHECK_PROGS(SPUCXX,spu-g++)
    AC_CHECK_PROGS(EMBEDSPU,["$host_alias"-embedspu embedspu])
    AC_CHECK_LIB(spe2, spe_context_create, have_libspe2=ok)
    test "x$SPUCC" != "x" \
      -a "x$have_libspe2" != "x" \
      -a "x$EMBEDSPU" != "x" && with_cell=yes
  fi
fi
if test "x$with_cell" = xyes ; then
  AC_MSG_CHECKING(whether $SPUCXX requires -ffreestanding option)
  AC_LANG_CONFTEST(
    int main(unsigned long long spe_id, unsigned long long pdata) {
      return 0;
    }
  )
  with_freestanding=
  ac_compile="$SPUCXX -c conftest.cpp"
  if eval "$ac_compile" 2>conftest.err ; then
    with_freestanding=no
  else
    with_freestanding=yes
    SPUCXX="$SPUCXX -ffreestanding"
  fi
  AC_MSG_RESULT($with_freestanding)
  AC_DEFINE([HAVE_LIBCELL], [1],
            [Defined if you have the Cell Broadband Engine environment])
  AC_SUBST([INCCELL], [""])
  AC_SUBST([LIBCELL], ["-lspe2 -lpthread"])
else
  with_cell=no
fi
AC_MSG_CHECKING(for Cell SDK)
AC_MSG_RESULT($with_cell)
test "x$with_cell" = xyes || AC_MSG_FAILURE( [Cell SDK not found.
  When cross-compiling, please indicate the target architecture, like:
    * --host=ppu for the 64-bit powerpc platform
    * --host=ppu32 for the 32-bit powerpc platform])

# Checks for the SDL library.
AC_CHECK_PROGS(SDL_CONFIG,sdl-config)
AC_MSG_CHECKING(for SDL)
AS_IF([test "x$with_sdl" != xno],
 [if test "x$with_sdl" = xyes -a "x$cross_compiling" = xyes; then
    AC_MSG_FAILURE(SDL support not available when cross-compiling)
  fi
  INCSDL=
  LIBSDL=
  with_sdl=no
  test "$ac_cv_prog_SDL_CONFIG" && with_sdl=yes
  if test "x$with_sdl" != xno; then
    AC_SUBST([INCSDL], ["`sdl-config --cflags`"])
    AC_SUBST([LIBSDL], ["`sdl-config --libs`"])
    AC_DEFINE([HAVE_SDL], [1], [Defined if you have libsdl])
  fi
 ])
AC_MSG_RESULT($with_sdl)

# Checks for the PS3FB kernel headers.
AC_MSG_CHECKING(for PS3FB kernel headers)
AS_IF([test "x$with_ps3fb" != xno],
 [if test "x$with_ps3fb" = xyes -a "x$ac_cv_header_asm_ps3fb_h" = xno; then
    AC_MSG_FAILURE("No PS3FB headers found")
  fi
  with_ps3fb=$ac_cv_header_asm_ps3fb_h
 ])
AC_MSG_RESULT($with_ps3fb)

AM_CONDITIONAL(USE_SDL, test "x$with_sdl" != "xno" )
AM_CONDITIONAL(USE_PS3FB, test "x$with_ps3fb" != "xno" )

AC_OUTPUT(Makefile \
          freezer/Makefile
          freezer/spu/Makefile
          scripts/Makefile
          examples/Makefile)
