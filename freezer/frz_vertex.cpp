/* Vertex classes.
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

#include <math.h>

#include "frz_vertex.hpp"

namespace Frz {

float vertex::norm2(void) {
  return x*x + y*y + z*z;
}

float vertex::norm(void) {
  return sqrtf(norm2());
}

void vertex::normalize(void) {
  float norm1 = 1./norm();
  x *= norm1;
  y *= norm1;
  z *= norm1;
}

/* Update the normal vector in vertex b of triangle (a,b,c).
 * This corresponds to adding the normalized ab * bc vector product,
 * multiplied by the angle in b */
void vertex::update_norm(const vertex &a, const vertex &b, const vertex &c) {
  vertex v1, v2, p;
  float norm, surf, angle;
  v1 = b-a;
  v2 = c-b;
  p = v1*v2;
  surf = sqrtf((v1.x * v1.x + v1.y * v1.y + v1.z * v1.z)
      * (v2.x * v2.x + v2.y * v2.y + v2.z * v2.z));
  norm = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
  if (norm > surf)
    angle = M_PI / 2;
  else
    angle = asinf(norm / surf);
  angle /= norm;
  x += p.x * angle;
  y += p.y * angle;
  z += p.z * angle;
}

svertex::svertex(const vertex &v) {
  int k;
  k = int(v.x*0x8000); x = (k>=0x8000) ? 0x7fff : k;
  k = int(v.y*0x8000); y = (k>=0x8000) ? 0x7fff : k;
  k = int(v.z*0x8000); z = (k>=0x8000) ? 0x7fff : k;
  k = int(v.t*0x8000); t = (k>=0x8000) ? 0x7fff : k;
}

} // namespace Frz
