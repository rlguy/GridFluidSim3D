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


