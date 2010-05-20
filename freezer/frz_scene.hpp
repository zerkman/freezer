/* Scene rendering data structures.
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

#ifndef _FRZ_SCENE_HPP_
#define _FRZ_SCENE_HPP_

#include <stdint.h>

#include <altivec.h>

#include "frz_obj3d.hpp"
#include "frz_misc.hpp"
#include "frz_defs.h"
#include "frz_vertex.hpp"

namespace Frz {

struct vvertex : public vertex {
  vvertex(): vertex() {}
  vvertex(const Vertex &v): vertex(v.x, v.y, v.z, 1.0f) {}
  vvertex(const vertex &v): vertex(v) {}
  vertex &as_vertex() const { return *(vertex*)this; }
  void update_norm(const vvertex &a, const vvertex &b, const vvertex &c) {
    vertex::update_norm(a, b, c);
  }
};

/* container class for 3D Triangle/Quad/Procedural data */
struct Triangle {
  /* Engine object type (cf. frz_defs.h) */
  uint16_t ob_type;
  /* Engine shading type (cf. frz_defs.h) */
  uint16_t sh_type;
  /* Rotation matrix number */
  uint16_t rot_id;
  /* Texture number */
  uint16_t tex_id;
  /* Flags */
  uint16_t flags;
  /* thickness (for extrusion) */
  uint16_t thickness;
  /* Color (for flat and smooth shading) */
  uint32_t color;

  vertex dummy;

  /* Triangle/Quad corners + mapping (u,v)'s */
  vertex a, b, c, d;
  /* Triangle normals */
  svertex na, nb, nc, nd;

  Triangle(uint16_t _ob_type, uint16_t _sh_type, uint16_t _rot_id,
      uint16_t _tex_id, uint32_t _color, const vvertex &_a, const vvertex &_b,
      const vvertex &_c, const vvertex &_d=vvertex()):
      ob_type(_ob_type), sh_type(_sh_type), rot_id(_rot_id), tex_id(_tex_id),
      flags(0), thickness(0), color(_color), 
      a(_a), b(_b), c(_c), d(_d), na(), nb(), nc(), nd() {}

  void setThickness(uint16_t th);
  void setFlatNormal(bool fn=true);
  void setDoubleSided(bool ds=true);
  void flatNormal();
};

class Scene {
  alignvec<Triangle> triangles;
  alignvec<Triangle> shared_triangles;
  vertex lightsource;
  uint16_t n_obj;
  int n_tri;
  int n_shared_tri;
  bool f_flash;
  uint32_t background;
  uint32_t filter_color;
  float filter_strength;

protected:
  void flash() {
    f_flash = true;
  }
  void setBackgroundColor(uint32_t c) {
    background = c;
  }
  void setFilter(uint32_t c, float s) {
    filter_color = c;
    filter_strength = s;
  }

public:
  Scene();
  virtual ~Scene();

  int add_triangle(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
      uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c);
  int add_quad(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
      uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c,
      const vertex &_d);
  int add_shared(uint16_t _ob_type, uint16_t _sh_type, uint16_t _rot_id,
      uint16_t _tex_id, uint32_t _color, const vertex &_a, const vertex &_b,
      const vertex &_c, const vertex &_d);
  int add_object(const Object3d &obj, uint16_t type=SH_SMOOTH, float zoom=1.0f,
      uint16_t tex_id=-1);
  int add_object_extrude(const Object3d &obj, uint16_t th, float zoom=1.0f);
  int add_object();
  virtual void setupFrame(uint32_t time, trans t[]) = 0;
  Triangle *getTriangles() { return &triangles[0]; }
  Triangle *getSharedTriangles() { return &shared_triangles[0]; }
  int getTriangleCount() { return n_tri; }
  int getSharedTriangleCount() { return n_shared_tri; }
  void setTriangleCount(int n) { n_tri = n; }
  int getObjectCount() { return n_obj; }
  const vertex &getLightSource() const { return lightsource; }
  bool getFlash() {
    bool f = f_flash; 
    f_flash = false;
    return f;
  }
  uint32_t getBackgroundColor() const { return background; }
  uint32_t getFilterColor() const { return filter_color; }
  float getFilterStrength() const { return filter_strength; }
};

}; // namespace Frz


#endif

