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

#include "frz_scene.hpp"

using namespace Frz;

Scene::Scene(): triangles(), shared_triangles(),
    lightsource(0.57735027f, -0.57735027f, -0.57735027f),
    n_obj(0), n_tri(0), n_shared_tri(0), f_flash(false), background(0x200020),
    filter_color(0xffffff), filter_strength(0.0f) {
}

Scene::~Scene() {
}

void Triangle::setThickness(uint16_t th) {
  if (th) {
    flags |= FL_EXTR;
  } else {
    flags &= ~FL_EXTR;
  }
  thickness = th;
}

void Triangle::setFlatNormal(bool fn) {
  if (fn) {
    flags |= FL_FLATN;
  } else {
    flags &= ~FL_FLATN;
  }
}

void Triangle::setDoubleSided(bool ds) {
  if (ds) {
    flags |= FL_DS;
  } else {
    flags &= ~FL_DS;
  }
}

void Triangle::flatNormal() {
  vertex u, v, n;
  u = b-a;
  v = c-b;
  n = u*v;

  n.normalize();
  n.t = 0.f;

  na = nb = nc = nd = n;
}

int Scene::add_triangle(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
    uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c) {
  int i = triangles.growup();
  triangles[i] = Triangle(OB_TRIANGLE,
      _sh_type, _rot_id, _tex_id, _color, _a, _b, _c);
  n_tri = triangles.getSize();
  return i;
}

int Scene::add_quad(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
    uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c,
    const vertex &_d) {
  int i = triangles.growup();
  triangles[i] = Triangle(OB_QUAD,
      _sh_type, _rot_id, _tex_id, _color, _a, _b, _c, _d);
  n_tri = triangles.getSize();
  return i;
}

int Scene::add_shared(uint16_t _ob_type, uint16_t _sh_type, uint16_t _rot_id,
    uint16_t _tex_id, uint32_t _color, const vertex &_a, const vertex &_b,
    const vertex &_c, const vertex &_d) {
  int i = shared_triangles.growup();
  shared_triangles[i] = Triangle(_ob_type,
      _sh_type, _rot_id, _tex_id, _color, _a, _b, _c, _d);
  n_shared_tri = shared_triangles.getSize();
  return i;
}

int Scene::add_object(const Object3d &obj, uint16_t type, float zoom,
    uint16_t tex_id) {
  int i, nt, np;
  int t0 = triangles.getSize();
  if (n_obj+1 == MAX_OBJ) {
    std::cerr<< "Maximum object count reached (" << MAX_OBJ << ")" << std::endl;
    exit(1);
  }
  np = obj.polyCount();
  for (i=0; i<np; i++) {
    const Object3d::Poly &p = obj.poly(i);
    if (p.triangle)
      triangles[triangles.growup()] = Triangle(OB_TRIANGLE, type, n_obj, tex_id,
          p.color, obj.vert(p.a)*zoom, obj.vert(p.b)*zoom, obj.vert(p.c)*zoom);
    else
      triangles[triangles.growup()] = Triangle(OB_QUAD, type, n_obj, tex_id,
          p.color, obj.vert(p.a)*zoom, obj.vert(p.b)*zoom, obj.vert(p.c)*zoom,
          obj.vert(p.d)*zoom);
  }

  /* update normals */
  if (type == SH_FLAT) {
    nt = triangles.getSize();
    for (i=t0; i<nt; i++)
      triangles[i].flatNormal();
  }
  else if (type == SH_SMOOTH || type == SH_RGBSMOOTH) {
    int nv = obj.vertexCount();
    vvertex n[nv];
    int t1;
    memset(n, 0, nv*sizeof(vertex));
    for (i=0; i<np; i++) {
      const Object3d::Poly &p = obj.poly(i);
      if (p.triangle) {
        n[p.a].update_norm(obj.vert(p.c), obj.vert(p.a), obj.vert(p.b));
        n[p.b].update_norm(obj.vert(p.a), obj.vert(p.b), obj.vert(p.c));
        n[p.c].update_norm(obj.vert(p.b), obj.vert(p.c), obj.vert(p.a));
      } else {
        n[p.a].update_norm(obj.vert(p.d), obj.vert(p.a), obj.vert(p.b));
        n[p.b].update_norm(obj.vert(p.a), obj.vert(p.b), obj.vert(p.c));
        n[p.c].update_norm(obj.vert(p.b), obj.vert(p.c), obj.vert(p.d));
        n[p.d].update_norm(obj.vert(p.c), obj.vert(p.d), obj.vert(p.a));
      }
    }
    for (i=0; i<nv; i++)
      n[i].normalize();
    t1 = t0;
    for (i=0; i<np; i++) {
      const Object3d::Poly &p = obj.poly(i);
      Triangle &t = triangles[t1++];
      t.na = n[p.a];
      t.nb = n[p.b];
      t.nc = n[p.c];
      if (!p.triangle)
	t.nd = n[p.d];
    }
  }

  n_tri = triangles.getSize();
  return n_obj++;
}

int Scene::add_object_extrude(const Object3d &obj, uint16_t th, float zoom) {
  int n = getTriangleCount();
  int ret = add_object(obj, SH_FLAT, zoom);
  Triangle *t = getTriangles() + n;
  n = getTriangleCount() - n;
  for (int i=0; i<n; i++)
    t[i].setThickness(th);
  n_tri = triangles.getSize();
  return ret;
}


int Scene::add_object() {
  return n_obj++;
}
