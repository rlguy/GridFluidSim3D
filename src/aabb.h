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

#ifndef AABB_H
#define AABB_H

#include <math.h>
#include <vector>
#include "triangle.h"
#include "array3d.h"
#include "glm/glm.hpp"

class AABB
{
public:
    AABB();
    AABB(double x, double y, double z, double width, double height, double depth);
    AABB(glm::vec3 p, double width, double height, double depth);
    AABB(glm::vec3 p1, glm::vec3 p2);
    AABB(std::vector<glm::vec3> &points);
    AABB(Triangle t, std::vector<glm::vec3> &vertices);
    AABB(GridIndex g, double dx);
    ~AABB();

    void expand(double v);
    bool isPointInside(glm::vec3 p);
    void getOverlappingGridCells(double dx, std::vector<GridIndex> &cells);
    bool isOverlappingTriangle(Triangle t, std::vector<glm::vec3> &vertices);
    bool isLineIntersecting(glm::vec3 p1, glm::vec3 p2);

    glm::vec3 getMinPoint();
    glm::vec3 getMaxPoint();

    glm::vec3 position;
    double width, height, depth;

private:
    GridIndex _positionToGridIndex(glm::vec3 p, double dx);
    bool _axisTestX01(glm::vec3 v0, glm::vec3 v2,
        double a, double b, double fa, double fb);
    bool _axisTestX2(glm::vec3 v0, glm::vec3 v1,
        double a, double b, double fa, double fb);
    bool _axisTestY02(glm::vec3 v0, glm::vec3 v2,
        double a, double b, double fa, double fb);
    bool _axisTestY1(glm::vec3 v0, glm::vec3 v1,
        double a, double b, double fa, double fb);
    bool _axisTestZ12(glm::vec3 v1, glm::vec3 v2,
        double a, double b, double fa, double fb);
    bool _axisTestZ0(glm::vec3 v0, glm::vec3 v1,
        double a, double b, double fa, double fb);
    void _findminmax(double v0, double v1, double v2, double *min, double *max);
    bool _planeBoxOverlap(glm::vec3 normal, glm::vec3 vert);
};

#endif
