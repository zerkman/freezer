/* Animation manager.
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

#ifndef _FRZ_SCRIPT_HPP_
#define _FRZ_SCRIPT_HPP_

#include "frz_scene.hpp"

namespace Frz {

/*! \brief An animation manager.
 *
 * This class is responsible for managing a full animation. It allocates and
 * deallocates the different scenes, and decides which scene to render at
 * what time.
 */
class Script {
  friend class System;

  Scene *scene;

  void sceneSetup(uint32_t time, trans t[]) { scene->setupFrame(time, t); }
  Scene::Triangle *getTriangles() { return scene->getTriangles(); }
  int getTriangleCount() { return scene->getTriangleCount(); }
  //Scene::Triangle *getSharedTriangles() { return scene->getSharedTriangles(); }
  //int getSharedTriangleCount() { return scene->getSharedTriangleCount(); }
  int getObjectCount() { return scene->getObjectCount(); }
  const vertex &getLightSource() const { return scene->getLightSource(); }
  bool getFlash() { return scene->getFlash(); }
  uint32_t getBackgroundColor() { return scene->getBackgroundColor(); }
  uint32_t getFadeColor() { return scene->getFadeColor(); }
  float getFadeStrength() { return scene->getFadeStrength(); }

public:
  //! Constructor.
  Script() {}
  //! Destructor.
  virtual ~Script() {}

protected:
  /*! \brief Sets the current scene.
   *
   * \param s A pointer to the Scene object to be used.
   */
  void setScene(Scene *s) { scene = s; }

  /*! \brief Sets up an animation frame.
   *
   * This method is called before the \ref Scene::setupFrame "setupFrame"
   * method is called on the current scene. This gives the opportunity to
   * the script to switch the current scene according to the time value.
   *
   * \param time the animation time in milliseconds
   */
  virtual void setupFrame(uint32_t time) = 0;
};

}; // namespace Frz

#endif
