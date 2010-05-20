/* Global defines.
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

#ifndef _FRZ_DEFS_H_
#define _FRZ_DEFS_H_

#define MAX_OBJ 128

/* shading types */
#define SH_CONST 1
#define SH_FLAT 2
#define SH_SMOOTH 3
#define SH_RGBSMOOTH 4

/* engine object types */
#define OB_TRIANGLE 1
#define OB_QUAD 2

/* engine object flags */
#define FL_DS 1       /* Double sided poly */
#define FL_FLATN 2    /* Generate flat normals */
#define FL_EXTR 4     /* Perform extrusion */

#endif
