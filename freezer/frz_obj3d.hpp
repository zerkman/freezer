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

/*! \brief A 3D object.
 *
 * This class modelizes a 3D object. An object is made of:
 *  - a set of \ref Vertex "vertices";
 *  - a set of \ref Poly "polygons" defined by:
 *    - a list of 3 or 4 vertices encoding either a triangle or a quad;
 *    - a color value.
 */
class Object3d {
public:
/*! \brief A 3D vertex.
 */
struct Vertex {
  //! \e x coordinate
  float x;
  //! \e y coordinate
  float y;
  //! \e z coordinate
  float z;

  /*! \brief Constructor
   *
   * \param _x \e x coordinate
   * \param _y \e y coordinate
   * \param _z \e z coordinate
   */
  Vertex(float _x=0.f, float _y=0.f, float _z=0.f) : x(_x), y(_y), z(_z) {}

  /*! \brief Subtracts another Vertex.
   *
   * \param a a reference to the right hand argument of the subtraction
   *
   * \return the result of the subtraction
   */
  Vertex operator- (const Vertex &a) const {
    return Vertex(x-a.x, y-a.y, z-a.z);
  }

  /*! \brief Performs a cross product with vertex \p b.
   *
   * \param b a reference to the right hand argument of the cross product
   *
   * \return the result of the cross product
   */
  Vertex operator* (const Vertex &b) const {
    return Vertex(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x);
  }

  /*! \brief Multiplies all coefficients with scalar \p a.
   *
   * \param a the scalar multiplier
   *
   * \return the result of the operation
   */
  Vertex operator* (const float a) const {
    return Vertex(x*a, y*a, z*a);
  }

  /*! \brief Equality comparison with 10<sup>-5</sup> precision.
   *
   * \param a the vertex to be compared
   *
   * \return \b true if all pairs of coefficients differ by less
   *         than 10<sup>-5</sup>, and \b false otherwise
   */
  bool operator== (const Vertex &a) const {
    const double precision = 1e-5f;
    bool ret = fabs((a.x - x)) < precision &&
               fabs((a.y - y)) < precision &&
               fabs((a.z - z)) < precision;
    //std::cout << *this << (ret?" == ":" != ") << a << std::endl;
    return ret;
  }

  /*! \brief Writes the Vertex contents into an output stream.
   *
   * \param out a reference to the output stream
   */
  void write(std::ostream &out) {
    out.write((char*)&x, 12);
  }

  /*! \brief Reads the Vertex contents from an input stream.
   *
   * \param in a reference to the output stream
   */
  void read(std::istream &in) {
    in.read((char*)&x, 12);
  }

  /*! \brief Displays a Vertex in human-readable fashion.
   *
   * \param out a reference to the output stream
   * \param v a reference to the Vertex to be written
   *
   * \return a reference to the output stream
   */
  friend std::ostream &operator<< (std::ostream &out, const Vertex &v) {
    return out << v.x << " " << v.y << " " << v.z;
  }

  /*! \brief Inputs a Vertex written in human-readable fashion.
   *
   * \param in a reference to the input stream
   * \param v a reference to the Vertex to be read into
   *
   * \return a reference to the input stream
   */
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

/*! \brief A 3D polygon.
 *
 * A Poly object either stores a triangle or a quad. The polygon is defined
 * by a list of indices in an \ref Object3d "object"'s
 * \ref Object3d::vertices "vertex list".
 */
struct Poly {
  union {
    struct {
      int a;       //!< First vertex index.
      int b;       //!< Second vertex index.
      int c;       //!< Third vertex index.
      int d;       //!< Fourth vertex index.
    };
    int v[4];      //!< An array containing the vertex indices.
  };
  uint32_t color;  //!< The color for the polygon.
  /*! \brief An indicator containing \b true if the polygon is a triangle,
   * and \b false if it is a quad. */
  bool triangle;

  /*! \brief Constructor.
   *
   * \param _a First vertex index
   * \param _b Second vertex index
   * \param _c Third vertex index
   * \param _d Fourth vertex index (set to -1 if the poly is a triangle)
   * \param _color Polygon color in ARGB binary format
   */
  Poly(int _a=-1, int _b=-1, int _c=-1, int _d=-1, int _color=0xffffff);

  /*! \brief Compares two polygons.
   *
   * All possible rotations are tested for equality comparison.
   *
   * \return \b true if the polygons are equal, \b false otherwise
   */
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

  /*! \brief Writes a polygon into an output stream.
   *
   * \param out a reference to the output stream
   */
  void write(std::ostream &out) {
    out.write((char*)&a, 20);
  }

  /*! \brief Reads a polygon from an input stream.
   *
   * \param in a reference to the input stream
   */
  void read(std::istream &in) {
    in.read((char*)&a, 20);
    triangle = (d == -1);
  }

  /*! \brief Displays a polygon in human-readable fashion.
   *
   * \param out a reference to the output stream
   * \param q a reference to the polygon to be displayed
   *
   * \return a reference to the output stream
   */
  friend std::ostream &operator<< (std::ostream &out, const Poly &q) {
    if (q.triangle)
      return out << "3 " << q.a << " " << q.b << " " << q.c;
    else
      return out << "4 " << q.a << " " << q.b << " " << q.c << " " << q.d;
  }
};

protected:
  //! The internal vertex list.
  std::vector<Vertex> vertices;
  //! The internal polygon list.
  std::vector<Poly> polys;

public:
  //! Default constructor.
  Object3d() : vertices(), polys() {}
  //! Copy constructor with zoom.
  Object3d(const Object3d &obj, float zoom=1.0f);
  /*! \brief Constructor which reads an object file in Object File Format.
   *
   * \param filename the file name
   * \param zoom zoom factor to apply to the object's vertices
   * \param color a color value for the object (ARGB binary format)
   */
  Object3d(const std::string &filename, float zoom=100.0f,
      uint32_t color=0xffffff);
  //! Destructor.
  virtual ~Object3d() {}

  //! Returns the number of vertices.
  int vertexCount() const { return vertices.size(); }
  //! Returns the number of polygons.
  int polyCount() const { return polys.size(); }
  //! Returns a reference to vertex number \p i.
  const Vertex &vert(int i) const { return vertices[i]; }
  //! Returns a reference to polygon number \p i.
  const Poly &poly(int i) const { return polys[i]; }

  /*! \brief Adds a vertex.
   *
   * \param v a reference of the vertex whose copy is to be inserted
   *
   * \return the inserted vertex number
   */
  int addVertex(const Vertex &v);

  /*! \brief Adds a vertex.
   *
   * \param x <em>x</em>-coordinate of the new vertex to be inserted
   * \param y <em>y</em>-coordinate of the new vertex to be inserted
   * \param z <em>z</em>-coordinate of the new vertex to be inserted
   *
   * \return the inserted vertex number
   */
  int addVertex(float x, float y, float z);

  /*! \brief Inserts a polygon.
   *
   * \param a First vertex index
   * \param b Second vertex index
   * \param c Third vertex index
   * \param d Fourth vertex index (set to -1 if the poly is a triangle)
   * \param color Polygon color in ARGB binary format
   *
   * \return the inserted polygon number
   */
  int addPoly(int a, int b, int c, int d=-1, uint32_t color=0xffffff);

  /*! \brief Translates the whole object.
   *
   * \param x <em>x</em>-coordinate of the translation vector
   * \param y <em>y</em>-coordinate of the translation vector
   * \param z <em>z</em>-coordinate of the translation vector
   */
  void translate(float x, float y, float z);

  /*! \brief Merges another object.
   *
   * All of the objects's vertices and polygons are added into this object.
   *
   * \param obj a reference to the object to be merged
   */
  void merge(const Object3d &obj);

  /*! \brief Imports an object from internal binary file format.
   *
   * The internal binary file format is designed for speed and efficiency, and
   * contains exactly the contents of an Object3d object.
   *
   * \param filename the file name
   * \param zoom the zoom value
   */
  void importBin(const std::string &filename, float zoom=1.0f);

  /*! \brief Imports an object from Object File Format (OFF).
   *
   * OFF is a commonly used file format.
   *
   * \param filename the file name
   * \param zoom the zoom value
   * \param color Object color in ARGB binary format
   */
  void importOff(const std::string &filename, float zoom=1.0f,
      uint32_t color=0xffffff);

  /*! \brief Exports an object to internal binary file format.
   *
   * The internal binary file format is designed for speed and efficiency, and
   * contains exactly the contents of an Object3d object.
   *
   * \param filename the file name
   */
  void exportBin(const std::string &filename);

  /*! \brief Exports an object to Object File Format (OFF).
   *
   * OFF is a commonly used file format.
   *
   * \param filename the file name
   */
  void exportOff(const std::string &filename);
};

}; // namespace Frz

#endif
