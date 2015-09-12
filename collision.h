/*
Copyright (c) 2015 Ryan L. Guy

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#pragma once

#include <stdio.h>
#include <iostream>
#include "glm/glm.hpp"

namespace Collision {

    double _clamp(double v, double min, double max);

    // method adapted from:
    // http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
    extern bool rayIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                      glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, 
                                      glm::vec3 *collision, double *u, double *v);

    extern bool lineIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                       glm::vec3 v0, glm::vec3 v1, glm::vec3 v2,
                                       glm::vec3 *collision, double *u, double *v);

    extern bool rayIntersectsPlane(glm::vec3 p0, glm::vec3 dir,
                                   glm::vec3 planePoint, glm::vec3 planeNormal,
                                   glm::vec3 *collision);

    extern bool lineIntersectsPlane(glm::vec3 p0, glm::vec3 dir,
                                    glm::vec3 planePoint, glm::vec3 planeNormal,
                                    glm::vec3 *collision);


    // method adapted from:
    // http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
    extern glm::vec3 findClosestPointOnTriangle(glm::vec3 p0, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2);


    extern inline bool rayIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                             glm::vec3 v0, glm::vec3 v1, glm::vec3 v2);

    extern inline bool rayIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                             glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 *collision);

    extern inline bool lineIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                              glm::vec3 v0, glm::vec3 v1, glm::vec3 v2);

    extern inline bool lineIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                              glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 *collision);

    extern bool rayIntersectsPlane(glm::vec3 p0, glm::vec3 dir,
                                   glm::vec3 planePoint, glm::vec3 planeNormal);

    extern bool lineIntersectsPlane(glm::vec3 p0, glm::vec3 dir,
                                    glm::vec3 planePoint, glm::vec3 planeNormal);
   


}


