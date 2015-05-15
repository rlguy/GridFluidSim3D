#pragma once

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

