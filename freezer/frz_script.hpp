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

class Script {
  Scene *scene;
  bool f_flash;

protected:
  void setScene(Scene *s) {
    scene = s;
  }
  void sceneSetup(uint32_t time, trans t[]) {
    scene->setupFrame(time, t);
  }

public:
  Script(): f_flash(false) {}
  virtual ~Script() {}

  virtual void setupFrame(uint32_t time, trans t[]) = 0;
  Scene::Triangle *getTriangles() { return scene->getTriangles(); }
  int getTriangleCount() { return scene->getTriangleCount(); }
  Scene::Triangle *getSharedTriangles() { return scene->getSharedTriangles(); }
  int getSharedTriangleCount() { return scene->getSharedTriangleCount(); }
  int getObjectCount() { return scene->getObjectCount(); }
  const vertex &getLightSource() const { return scene->getLightSource(); }
  bool getFlash() { return scene->getFlash(); }
  uint32_t getBackgroundColor() { return scene->getBackgroundColor(); }
  uint32_t getFilterColor() { return scene->getFilterColor(); }
  float getFilterStrength() { return scene->getFilterStrength(); }
};

}; // namespace Frz

#endif
