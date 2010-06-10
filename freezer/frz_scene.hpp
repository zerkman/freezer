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

/*! \brief Base class for animated 3D scenes.
 *
 * This is the base class for programming an animated 3D scene.
 *
 * \par
 * The Scene class basically embeds an array of 3D polygons (triangles or
 * quads), organized to make up a complete scene. Each polygon is
 * space-located according to its vertices, whose coordinates are modified by
 * a rotation/translation matrix. One can consider all polygons sharing the
 * same rotation/translation matrix to form a 3D object, since they are
 * &ldquo;tied&rdquo; together.
 *
 * \par Design your own scene
 * A Scene-derived class must populate the polygon array in its constructor.
 * Then, the animation system will call the \ref Scene::setupFrame "setupFrame"
 * method of your derived class before each time new frame is rendered. In this
 * method, you have to set the proper values to generate the
 * rotation/translation matrices, you can also change the screen background
 * color, and play with the fade and flash effects.
 *
 * \par
 * The following example is taken from the \c cube.cpp example program file
 * in the \c examples directory of the Freezer source archive.
 * \code
// An example scene with a rotating cube
class ScnCube : public Frz::Scene {
public:
  ScnCube() {
    int16_t r = 100;
    Frz::Object3d obj;
    // The eight vertices for the cube
    obj.addVertex(-r, -r, -r);  // vertex 0
    obj.addVertex( r, -r, -r);  // vertex 1
    obj.addVertex(-r,  r, -r);  // vertex 2
    obj.addVertex( r,  r, -r);  // vertex 3
    obj.addVertex(-r, -r,  r);  // vertex 4
    obj.addVertex( r, -r,  r);  // vertex 5
    obj.addVertex(-r,  r,  r);  // vertex 6
    obj.addVertex( r,  r,  r);  // vertex 7
    // The six faces defined according to the vertices above
    obj.addPoly(6, 7, 5, 4, 0x00f0f0);
    obj.addPoly(0, 1, 3, 2, 0xf000f0);
    obj.addPoly(2, 6, 4, 0, 0xf0f000);
    obj.addPoly(1, 5, 7, 3, 0xf00000);
    obj.addPoly(2, 3, 7, 6, 0x00f000);
    obj.addPoly(0, 4, 5, 1, 0x0000f0);

    add_object(obj);  // Adds the cube object into the scene. Its
                      // rotating/translation matrix identifier is zero as it
                      // is the first object inserted.
  }

  // This method is called before rendering a new frame
  virtual void setupFrame(uint32_t time, Frz::trans transforms[]) {
    transforms[0].a = time * 0.001f;  // Rotation angle according to the x axis
    transforms[0].b = time * 0.001f;  // Rotation angle according to the y axis
    transforms[0].c = time * 0.001f;  // Rotation angle according to the z axis
    transforms[0].x = 0.0f;           // Translation according to the x axis
    transforms[0].y = 0.0f;           // Translation according to the y axis
    transforms[0].z = -400.0f;        // Translation according to the z axis
  }
};
 * \endcode
 */
class Scene {
  friend class Script;

/*! \brief A \ref Frz::vertex "vertex" to Object3d::Vertex wrapper class.
 */
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
 * Triangle objects actually embed data for storing either a triangle or a
 * quad. The class contains the four vertices necessary to produce a quad,
 * where the fourth one is ignored when the class encodes a triangle.
 *
 * The class information includes:
 *  - the object type (triangle or quad);
 *  - the shading type;
 *  - a rotation matrix identifier;
 *  - a texture identifier (unused yet);
 *  - a color value;
 *  - specific flags-dependent data.
 *
 * For each vertex, the following information is provided:
 *  - its 3D coordinates (\e x, \e y, \e z);
 *  - the 3D coordinates (\e x, \e y, \e z) of the normal vector associated
 *    to the vertex:
 *  - a pair of (u,v) coordinates for texture mapping (unused yet).
 *
 * The Triangle class is designed in such a way that it exactly takes 128
 * bytes in memory. This is necessary to optimize PPE/SPE DMA communication.
 * The Scene class ensures its Triangle buffer is 128-byte aligned in memory
 * for that same purpose, using the specifically-designed \ref alignvec
 * aligned buffer.
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
   * \param _color 32-bit color value (ARGB binary format)
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

  /*! \brief Sets extrusion mode and thickness.
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
  /*! \brief Sets automatic normal vector generation for flat shading.
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

  /*! \brief Returns the light source direction.
   *
   * \return a normalized vector indicating the light source direction
   */
  const vertex &getLightSource() const { return lightsource; }

  /*! \brief Gets and resets the flash status.
   *
   * The flash status is set to \b false and the previous status is returned.
   *
   * \return \b true if the flash status was on before the call of the method
   */
  bool getFlash() {
    bool f = f_flash; 
    f_flash = false;
    return f;
  }

  /*! \brief Returns the background color.
   *
   * \return the background color in ARGB binary format
   */
  uint32_t getBackgroundColor() const { return background; }

  /*! \brief Returns the fade color.
   *
   * \return the fade color in ARGB binary format
   */
  uint32_t getFadeColor() const { return fade_color; }

  /*! \brief Returns the fade strength.
   *
   * \return the fade strength (between 0 and 1)
   */
  float getFadeStrength() const { return fade_strength; }

protected:
  /*! \brief Triggers a flash effect.
   *
   * The flash effect renders the next frame to uniform white, then the
   * subsequent frames quickly fade back to normal.
   */
  void flash() {
    f_flash = true;
  }
  /*! \brief Sets the background color.
   *
   * \param c the new background color (in ARGB binary format)
   */
  void setBackgroundColor(uint32_t c) {
    background = c;
  }
  /*! \brief Configures the fade effect.
   *
   * The fade effect consists, for each pixel in the generated picture, in a
   * linear interpolation of the fade color and the pixel. If the fade
   * strength is 0, the pixels are not modified. If it is 1, the full picture
   * is filled by the fade color.
   *
   * \param c the fade color (in ARGB binary format)
   * \param s the fade strength, between 0 (inactive) and 1 (full) 
   */
  void setFade(uint32_t c, float s) {
    fade_color = c;
    fade_strength = s;
  }
  /*! \brief Adds a triangle.
   *
   * \param _sh_type shading type (cf. frz_defs.h)
   * \param _rot_id rotation matrix number
   * \param _tex_id texture number (unused yet)
   * \param _color color (ARGB binary format)
   * \param _a first vertex
   * \param _b second vertex
   * \param _c third vertex
   * \return polygon index in internal array
   */
  int add_triangle(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
      uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c);

  /*! \brief Adds a quad.
   *
   * \param _sh_type shading type (cf. frz_defs.h)
   * \param _rot_id rotation matrix number
   * \param _tex_id texture number (unused yet)
   * \param _color color (ARGB binary format)
   * \param _a first vertex
   * \param _b second vertex
   * \param _c third vertex
   * \param _d fourth vertex
   * \return polygon index in internal array
   */
  int add_quad(uint16_t _sh_type, uint16_t _rot_id, uint16_t _tex_id,
      uint32_t _color, const vertex &_a, const vertex &_b, const vertex &_c,
      const vertex &_d);

  /* \brief Adds an element in shared list.
   *
   * This method is for internal purposes only.
   *
   * \param _ob_type object type (cf. frz_defs.h)
   * \param _sh_type shading type (cf. frz_defs.h)
   * \param _rot_id rotation matrix number
   * \param _tex_id texture number (unused yet)
   * \param _color color (ARGB binary format)
   * \param _a first vertex
   * \param _b second vertex
   * \param _c third vertex
   * \param _d fourth vertex
   * \return polygon index in internal array
   */
  /*int add_shared(uint16_t _ob_type, uint16_t _sh_type, uint16_t _rot_id,
      uint16_t _tex_id, uint32_t _color, const vertex &_a, const vertex &_b,
      const vertex &_c, const vertex &_d);*/

  /*! \brief Adds a 3D object.
   *
   * The method inserts all the polygons of the Object3d object into the scene.
   *
   * To each added object is assigned a new rotation matrix identifier, which is
   * the identifier of the previously allocated matrix identifier plus one. The
   * first identifier is zero.
   *
   * The normal vectors of each of the object's polygons are computed according
   * to the shading type.
   *
   * \param obj the 3D object to be added
   * \param type shading type (cf. frz_defs.h)
   * \param zoom value
   * \return the rotation matrix identifier
   */
  int add_object(const Object3d &obj, uint16_t type=SH_SMOOTH, float zoom=1.0f,
      uint16_t tex_id=-1);

  /*! \brief Adds a 3D object with polygon extrusion.
   *
   * Same operation as \ref add_object, except every polygon in the object is
   * extruded (see Triangle::setThickness).
   *
   * The shading type is flat, and the normal vectors are computed
   * automatically.
   *
   * \param obj the 3D object to be added
   * \param th thickness value
   * \param type shading type (cf. frz_defs.h)
   * \param zoom value
   * \return the rotation matrix identifier
   */
  int add_object_extrude(const Object3d &obj, uint16_t th, float zoom=1.0f);

  /*! \brief Allocates a rotation matrix identifier for a new object.
   *
   * This method is useful when adding polygons directly into the scene, using
   * \ref add_triangle and \ref add_quad, which require a valid rotation matrix
   * identifier.
   *
   * \return the rotation matrix identifier
   */
  int add_object();

  /*! \brief Returns a reference to the internal polygon array.
   *
   * \return a reference to the internal polygon array
   */
  Triangle *getTriangles() { return &triangles[0]; }

  /* \brief Returns a reference to the internal shared polygon array.
   *
   * This method is for internal purposes only.
   *
   * \return a reference to the internal polygon array
   */
  //Triangle *getSharedTriangles() { return &shared_triangles[0]; }

  /*! \brief Returns the number of elements in the internal polygon array.
   *
   * \return the number of elements in the internal polygon array.
   */
  int getTriangleCount() { return n_tri; }

  /* \brief Returns the number of elements in the internal shared polygon
   * array.
   *
   * This method is for internal purposes only.
   *
   * \return the number of elements in the internal shared polygon array.
   */
  //int getSharedTriangleCount() { return n_shared_tri; }

  /*! \brief Returns the number of objects (rotation matrices).
   *
   * \return the number of objects (rotation matrices)
   */
  int getObjectCount() { return n_obj; }

  /*! \brief Sets up a scene frame.
   *
   * This method is called by the animation system before drawing a new
   * frame. It is responsible for setting the correct animation values
   * (rotation and translation of every object in the scene, plus other
   * effects), according to the current animation time.
   *
   * \param time the animation time in milliseconds
   * \param t a set of rotation/translation structures to be populated
   */
  virtual void setupFrame(uint32_t time, trans t[]) = 0;

public:
  //! Constructor
  Scene();
  //! Destructor
  virtual ~Scene();
};

}; // namespace Frz


#endif

