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

class Scene {
  friend class Script;

struct vvertex : public vertex {
  vvertex(): vertex() {}
  vvertex(const Object3d::Vertex &v): vertex(v.x, v.y, v.z, 1.0f) {}
  vvertex(const vertex &v): vertex(v) {}
  vertex &as_vertex() const { return *(vertex*)this; }
  void update_norm(const vvertex &a, const vvertex &b, const vvertex &c) {
    vertex::update_norm(a, b, c);
  }
};

public:
/*! \brief Container class for 3D triangle or quad polygon data.
 *
 * Triangle objects actually embed data for storing either a triangle or a quad.
 * The class contains the four vertices necessary to produce a quad, where the
 * fourth one is ignored when the class encodes a triangle.
 *
 * The class information includes:
 *  - the object type (triangle or quad);
 *  - the shading type;
 *  - a rotation matrix identifier;
 *  - a color value;
 *  - specific flags-dependent data.
 *
 * For each vertex, the following information is provided:
 *  - its 3D coordinates (\e x, \e y, \e z);
 *  - the 3D coordinates (\e x, \e y, \e z) of the normal vector associated to
 *    the vertex.
 */
struct Triangle {
  //! Engine object type. (cf. frz_defs.h)
  uint16_t ob_type;
  //! Engine shading type. (cf. frz_defs.h)
  uint16_t sh_type;
  //! Rotation matrix number.
  uint16_t rot_id;
  //! Texture number. (not used yet)
  uint16_t tex_id;
  //! Flags.
  uint16_t flags;
  //! Thickness. (for extrusion)
  uint16_t thickness;
  //! Color. (for flat and smooth shading)
  uint32_t color;

  //! Dummy or flags-dependent data.
  vertex dummy;

  /*! \brief First Triangle/Quad vertex + mapping (u,v)
   *
   * The three first elements (\e x, \e y, and \e z) of this vertex are the 3D
   * \e x, \e y, and \e z coordinates of the triangle/quad vertex. The fourth
   * element is a pair of 16-bit fixed point mapping coordinates, where the \c
   * 0x0 value is 0 and \c 0xFFFF is 0.99998.
   */
  vertex a;
  /*! \brief Second Triangle/Quad vertex + mapping (u,v)
   *
   * The three first elements (\e x, \e y, and \e z) of this vertex are the 3D
   * \e x, \e y, and \e z coordinates of the triangle/quad vertex. The fourth
   * element is a pair of 16-bit fixed point mapping coordinates, where the \c
   * 0x0 value is 0 and \c 0xFFFF is 0.99998.
   */
  vertex b;
  /*! \brief Third Triangle/Quad vertex + mapping (u,v)
   *
   * The three first elements (\e x, \e y, and \e z) of this vertex are the 3D
   * \e x, \e y, and \e z coordinates of the triangle/quad vertex. The fourth
   * element is a pair of 16-bit fixed point mapping coordinates, where the \c
   * 0x0 value is 0 and \c 0xFFFF is 0.99998.
   */
  vertex c;
  /*! \brief Fourth Quad vertex + mapping (u,v)
   *
   * The three first elements (\e x, \e y, and \e z) of this vertex are the 3D
   * \e x, \e y, and \e z coordinates of the quad vertex. The fourth
   * element is a pair of 16-bit fixed point mapping coordinates, where the \c
   * 0x0 value is 0 and \c 0xFFFF is 0.99998.
   */
  vertex d;
  //! Normal vector at first vertex.
  svertex na;
  //! Normal vector at second vertex.
  svertex nb;
  //! Normal vector at third vertex.
  svertex nc;
  //! Normal vector at fourth vertex.
  svertex nd;

  /*! \brief Constructor for Triangle objects.
   *
   * \param _ob_type Object type. One of \c OB_TRIANGLE or \c OB_QUAD (see
   *                 frz_defs.h)
   * \param _sh_type Shading type
   * \param _rot_id Rotation matrix identifier
   * \param _tex_id Texture identifier (currently unused)
   * \param _color 32-bit color value (ARGB format)
   * \param _a first triangle/quad vertex
   * \param _b second triangle/quad vertex
   * \param _c third triangle/quad vertex
   * \param _d fourth quad vertex
   */
  Triangle(uint16_t _ob_type, uint16_t _sh_type, uint16_t _rot_id,
      uint16_t _tex_id, uint32_t _color, const vvertex &_a, const vvertex &_b,
      const vvertex &_c, const vvertex &_d=vvertex()):
      ob_type(_ob_type), sh_type(_sh_type), rot_id(_rot_id), tex_id(_tex_id),
      flags(0), thickness(0), color(_color), 
      a(_a), b(_b), c(_c), d(_d), na(), nb(), nc(), nd() {}

  /*! \brief Set extrusion mode and thickness.
   *
   * If \p th is zero, extrusion is deactivated. If it is nonzero, extrusion is
   * activated for this polygon, and the \p th value is used as thickness value.
   *
   * Polygon extrusion basically consists in generating an orthogonal
   * revolution object using the given polygon as a base. For instance, to
   * generate a cube, simply set the thickness of a square object to its edge
   * length.
   *
   * The generated object's faces have their normal vectors calculated for flat
   * shading. Normal vectors cannot be generated for any other shading type.
   *
   * \param th thickness value
   */
  void setThickness(uint16_t th);
  /*! \brief Set automatic normal vector generation for flat shading.
   *
   * If this mode is set, the engine automatically computes the normal vector
   * of the polygon at each frame update, and sets the computed value to the
   * normal vector at each vertex of the polygon.
   *
   * \param fn if \b true, automatic normal vector generation is activated, and
   *           is deactivatd otherwise
   */
  void setFlatNormal(bool fn=true);
  /*! \brief Sets the double sided-ness of the polygon.
   *
   * Double-sided polygons can be seen from both sides, in opposite of
   * single-sided polys which have only one visible side.
   *
   * Polygons are single-sided by default. In this case, the visible side is the
   * one for which the vertices appear in clockwise order.
   *
   * \param ds if \b true, the polygon is considered double-sided, otherwise it
   *           is not.
   */
  void setDoubleSided(bool ds=true);
  /*! \brief Sets the normal vectors for flat shading.
   */
  void flatNormal();
};

private:
  alignvec<Triangle> triangles;
  alignvec<Triangle> shared_triangles;
  vertex lightsource;
  uint16_t n_obj;
  int n_tri;
  int n_shared_tri;
  bool f_flash;
  uint32_t background;
  uint32_t fade_color;
  float fade_strength;

  const vertex &getLightSource() const { return lightsource; }
  bool getFlash() {
    bool f = f_flash; 
    f_flash = false;
    return f;
  }
  uint32_t getBackgroundColor() const { return background; }
  uint32_t getFadeColor() const { return fade_color; }
  float getFadeStrength() const { return fade_strength; }

protected:
  void flash() {
    f_flash = true;
  }
  void setBackgroundColor(uint32_t c) {
    background = c;
  }
  void setFade(uint32_t c, float s) {
    fade_color = c;
    fade_strength = s;
  }
  int add_triangle(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
      uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c);
  int add_quad(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
      uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c,
      const vertex &_d);
  /*int add_shared(uint16_t _ob_type, uint16_t _sh_type, uint16_t _rot_id,
      uint16_t _tex_id, uint32_t _color, const vertex &_a, const vertex &_b,
      const vertex &_c, const vertex &_d);*/
  int add_object(const Object3d &obj, uint16_t type=SH_SMOOTH, float zoom=1.0f,
      uint16_t tex_id=-1);
  int add_object_extrude(const Object3d &obj, uint16_t th, float zoom=1.0f);
  int add_object();
  Triangle *getTriangles() { return &triangles[0]; }
  //Triangle *getSharedTriangles() { return &shared_triangles[0]; }
  int getTriangleCount() { return n_tri; }
  //int getSharedTriangleCount() { return n_shared_tri; }
  int getObjectCount() { return n_obj; }
  virtual void setupFrame(uint32_t time, trans t[]) = 0;

public:
  Scene();
  virtual ~Scene();
};

}; // namespace Frz


#endif

