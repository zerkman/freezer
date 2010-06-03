/* 3D object data structures.
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

#include <fstream>
#include <iomanip>

#include "frz_obj3d.hpp"

using namespace Frz;

Object3d::Poly::Poly(int _a, int _b, int _c, int _d, int _color) : a(_a), b(_b),
    c(_c), d(_d), color(_color), triangle(false) {
  if (d == -1)
    triangle = true;
  else if (a == b) {
    b = c;
    c = d;
    d = -1;
    triangle = true;
  } else if (b == c) {
    c = d;
    d = -1;
    triangle = true;
  } else if (c == d || a == d) {
    d = -1;
    triangle = true;
  }
}

Object3d::Object3d(const Object3d &obj, float zoom) :
    vertices(obj.vertices), polys(obj.polys) {
  int i, nv = obj.vertexCount();
  Vertex *v;

  for (i=0; i<nv; i++) {
    v = &vertices[i];
    v->x *= zoom;
    v->y *= zoom;
    v->z *= zoom;
  }
}

Object3d::Object3d(const std::string &filename, float zoom, uint32_t color) :
    vertices(), polys() {
  importOff(filename, zoom, color);
}

int Object3d::addVertex(const Vertex &v) {
  int i = vertices.size();
  vertices.push_back(v);
  return i;
}

int Object3d::addVertex(float x, float y, float z) {
  Vertex v(x, y, z);
  return addVertex(v);
}

int Object3d::addPoly(int a, int b, int c, int d, uint32_t color) {
  int i = polys.size();
  Poly p(a, b, c, d, color);
  polys.push_back(p);
  return i;
}

void Object3d::translate(float x, float y, float z) {
  int i;
  for (i=0; i<(int)vertices.size(); i++) {
    vertices[i].x += x;
    vertices[i].y += y;
    vertices[i].z += z;
  }
}

void Object3d::merge(const Object3d &obj) {
  int vcount = obj.vertexCount();
  int pcount = obj.polyCount();
  int s = vertices.size();
  int i,j;
  for (i = 0; i < vcount; i++) {
    vertices.push_back(obj.vert(i));
  }
  for (i = 0; i < pcount; i++) {
    Poly p = obj.poly(i);
    for (j = 0; j < 4; j++)
      if (p.v[j] != -1)
        p.v[j] += s;
    polys.push_back(p);
  }
}

void Object3d::importBin(const std::string &filename, float zoom) {
  std::ifstream in(filename.c_str());
  int num_vertices, num_polys;
  in.read((char*)&num_vertices, sizeof(num_vertices));
  in.read((char*)&num_polys, sizeof(num_polys));
  vertices.resize(num_vertices);
  for (int i=0; i<num_vertices; i++) {
    vertices[i].read(in);
    vertices[i] = vertices[i]*zoom;
  }
  polys.resize(num_polys);
  for (int i=0; i<num_polys; i++)
    polys[i].read(in);
  in.close();
}

void Object3d::importOff(const std::string &filename, float zoom,
    uint32_t color) {
  std::ifstream in(filename.c_str());
  char temp[16];
  float x, y, z;
  int a, b, c, d;
  int nv, nq, i, dummy, count;
  in >> temp >> nv >> nq >> dummy;
  for (i=0; i<nv; i++) {
    in >> x >> y >> z;
    x *= zoom;
    y *= zoom;
    z *= zoom;
    addVertex(x, y, z);
  }
  for (i=0; i<nq; i++) {
    in >> count;
    if (count == 3) {
      in >> a >> b >> c;
      addPoly(a, b, c, -1, color);
    }
    else if (count == 4) {
      in >> a >> b >> c >> d;
      addPoly(a, b, c, d, color);
    }
    else {
      std::cout << "unsupported poly type:" << count << std::endl;
    }
  }
}

void Object3d::exportBin(const std::string &filename) {
  int num_vertices = vertices.size();
  int num_polys = polys.size();
  std::ofstream out(filename.c_str());
  out.write((char*)&num_vertices, sizeof(num_vertices));
  out.write((char*)&num_polys, sizeof(num_polys));
  for (int i=0; i<num_vertices; i++)
    vertices[i].write(out);
  for (int i=0; i<num_polys; i++)
    polys[i].write(out);
  out.close();
}

void Object3d::exportOff(const std::string &filename) {
  std::ofstream out(filename.c_str());
  int i, n;

  out << "OFF" << std::endl;
  out << vertices.size() << " " << polys.size() << " 0" << std::endl;
  n = vertices.size();
  for (i=0; i<n; i++)
    out << vertices[i] << std::endl;
  n = polys.size();
  for (i=0; i<n; i++) {
    out << polys[i] << std::endl;
  }
  out.close();
}
