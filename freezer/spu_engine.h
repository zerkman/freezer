/* SPU engine.
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

#ifndef __SPU_ENGINE_H__
#define __SPU_ENGINE_H__

#include "frz_defs.h"

#define LIST_COUNT 16

#ifndef __SPU__
typedef int qword __attribute__((vector_size(16)));
#else
struct list_buf;

void proj_render_poly(vec_ushort8 t0, vec_float4 t1,
    vec_float4 a, vec_float4 b, vec_float4 c, vec_float4 d,
    vec_float4 na, vec_float4 nb, vec_float4 nc, vec_float4 nd,
    struct list_buf *lst);
#endif

typedef struct 
{
  int spe_rank;
  int spe_count;
  unsigned int width;
  unsigned int height;
  unsigned int pitch;
  uint32_t __dummy[3];
  uint64_t sig1[6];
} spe_data;

typedef struct
{
  uint64_t e3d_ea;      /* address of effect class */
  uint64_t t_ea;        /* address of triangle array */
  uint64_t sh_t_ea;     /* address of shared special object array */
  uint64_t fd_ea;       /* address of frame data */
  uint32_t n_obj;       /* number of objects (=rotation matrices) */
  uint32_t n_t;         /* number of triangles */
  uint64_t sync_ea;     /* address of first synchronization buffer */
  uint32_t time;        /* realtime clock from program start (milliseconds) */
  uint32_t n_shared;    /* number of shared special objects */
  uint64_t __dummy;
  vector float lightsource;     /* light source direction */
  uint64_t td_ea[LIST_COUNT];   /* address of triangle data array */
#if LIST_COUNT&1
  uint64_t __dummy2;
#endif
} spe_3d_data;

typedef struct
{
  uint64_t screen_ea;   /* address of screen frame buffer */
  uint64_t oldscreen_ea;/* address of previous screen frame buffer */
  uint64_t data3d_ea;   /* address of 3D data structure */
  uint32_t backgdc;     /* background color */
  float blur_strength;  /* strength of motion blur */
  float blur_strength1; /* 1-strength of motion blur */
  float flash_strength; /* strength of flashing effect */
  uint32_t filter_color; /* colouring of flash effect */
  float filter_strength; /* strength of filter */
  /*uint32_t __dummy[2];*/
  vector float rot[MAX_OBJ][4];
} spe_frame_data;


#endif

