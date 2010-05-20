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
/*
 * frz_pixbuf.hpp - pixel buffer manager for the Freezer animation system
 * by François Galea <fgalea@free.fr>
 */

#ifndef __FRZ_OBJ3D_HPP__
#define __FRZ_OBJ3D_HPP__

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

#include <stdint.h>

namespace Frz {

struct Vertex {
  float x, y, z;

  Vertex(float _x=0.f, float _y=0.f, float _z=0.f) : x(_x), y(_y), z(_z) {}

  Vertex operator- (const Vertex &a) const {
    return Vertex(x-a.x, y-a.y, z-a.z);
  }

  Vertex operator* (const Vertex &b) const {
    return Vertex(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x);
  }

  Vertex operator* (const float a) const {
    return Vertex(x*a, y*a, z*a);
  }

  bool operator== (const Vertex &a) const {
    const double precision = 1e-5f;
    bool ret = fabs((a.x - x)) < precision &&
               fabs((a.y - y)) < precision &&
               fabs((a.z - z)) < precision;
    //std::cout << *this << (ret?" == ":" != ") << a << std::endl;
    return ret;
  }

  void write(std::ostream &out) {
    out.write((char*)&x, 12);
  }

  void read(std::istream &out) {
    out.read((char*)&x, 12);
  }

  friend std::ostream &operator<< (std::ostream &out, const Vertex &v) {
    return out << v.x << " " << v.y << " " << v.z;
  }

  friend std::istream &operator>> (std::istream &in, Vertex &v) {
    char buf[80], *p;
    in >> buf;
    p = buf;
    v.x = strtod(p, &p); p++;
    v.y = strtod(p, &p); p++;
    v.z = strtod(p, &p);
    return in;
  }
};

struct Poly {
  union {
    struct { int a, b, c, d; };
    int v[4];
  };
  uint32_t color;
  bool triangle;

  Poly(int _a=-1, int _b=-1, int _c=-1, int _d=-1, int _color=0xffffff);

  bool operator== (const Poly &p) {
    if (d==-1)
      return (a==p.a && b==p.b && c==p.c)
          || (b==p.a && c==p.b && a==p.c)
          || (c==p.a && a==p.b && b==p.c);
    else
      return (a==p.a && b==p.b && c==p.c && d==p.d)
          || (b==p.a && c==p.b && d==p.c && a==p.d)
          || (c==p.a && d==p.b && a==p.c && b==p.d)
          || (d==p.a && a==p.b && b==p.c && c==p.d);
  }

  void write(std::ostream &out) {
    out.write((char*)&a, 20);
  }

  void read(std::istream &out) {
    out.read((char*)&a, 20);
    triangle = (d == -1);
  }

  friend std::ostream &operator<< (std::ostream &out, const Poly &q) {
    if (q.triangle)
      return out << "3 " << q.a << " " << q.b << " " << q.c;
    else
      return out << "4 " << q.a << " " << q.b << " " << q.c << " " << q.d;
  }
};

class Object3d {
protected:
  std::vector<Vertex> vertices;
  std::vector<Poly> polys;

public:
  Object3d() : vertices(), polys() {}
  Object3d(const Object3d &obj, float zoom=1.0f);
  Object3d(const std::string &filename, float zoom=100.0f,
      uint32_t color=0xffffff);
  virtual ~Object3d() {}

  int vertexCount() const { return vertices.size(); }
  int polyCount() const { return polys.size(); }
  const Vertex &vert(int i) const { return vertices[i]; }
  const Poly &poly(int i) const { return polys[i]; }

  int addVertex(const Vertex &v);
  int addVertex(float x, float y, float z);
  int addPoly(int a, int b, int c, int d=-1, uint32_t color=0xffffff);
  void translate(float x, float y, float z);
  void merge(const Object3d &obj);

  void importBin(const std::string &filename, float zoom=1.0f);
  void importOff(const std::string &filename, float zoom=1.0f,
      uint32_t color=0xffffff);
  void exportBin(const std::string &filename);
  void exportOff(const std::string &filename);
};

}; // namespace Frz

#endif
