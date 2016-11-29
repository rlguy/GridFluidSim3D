/*
Copyright (c) 2016 Ryan L. Guy

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
#ifndef COLLISION_H
#define COLLISION_H

#include <stdio.h>
#include <iostream>

#include "vmath.h"
#include "grid3d.h"
#include "fluidmaterialgrid.h"
#include "aabb.h"
#include "fluidsimassert.h"

namespace Collision {

    double _clamp(double v, double min, double max);

    // method adapted from:
    // http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
    extern bool rayIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                      vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2, 
                                      vmath::vec3 *collision, double *u, double *v);

    extern bool lineIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                       vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2,
                                       vmath::vec3 *collision, double *u, double *v);

    extern bool rayIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir,
                                   vmath::vec3 planePoint, vmath::vec3 planeNormal,
                                   vmath::vec3 *collision);

    extern bool lineIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir,
                                    vmath::vec3 planePoint, vmath::vec3 planeNormal,
                                    vmath::vec3 *collision);


    // method adapted from:
    // http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
    extern vmath::vec3 findClosestPointOnTriangle(vmath::vec3 p0, vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2);


    extern bool rayIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                             vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2);

    extern bool rayIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                             vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2, vmath::vec3 *collision);

    extern bool lineIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                              vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2);

    extern bool lineIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                              vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2, vmath::vec3 *collision);

    extern bool rayIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir,
                                   vmath::vec3 planePoint, vmath::vec3 planeNormal);

    extern bool lineIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir,
                                    vmath::vec3 planePoint, vmath::vec3 planeNormal);
   
    extern vmath::vec3 getTriangleCentroid(vmath::vec3 p0, vmath::vec3 p1, vmath::vec3 p2);
    extern vmath::vec3 getTriangleNormal(vmath::vec3 p0, vmath::vec3 p1, vmath::vec3 p2);

    extern bool getLineSegmentVoxelIntersection(vmath::vec3 p0, 
                                                vmath::vec3 p1,
                                                double dx,
                                                FluidMaterialGrid &grid,
                                                GridIndex *voxel);

    extern bool rayIntersectsAABB(vmath::vec3 p0, vmath::vec3 dir,
                                  AABB &bbox, vmath::vec3 *collision);

    extern bool sphereIntersectsAABB(vmath::vec3 p, double r, AABB bbox);
}

#endif
