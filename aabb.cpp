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
#include "aabb.h"


AABB::AABB() : position(glm::vec3(0.0, 0.0, 0.0)),
               width(0.0), height(0.0), depth(0.0)
{
}

AABB::AABB(double x, double y, double z, double w, double h, double d) : 
               position(glm::vec3(x, y, z)), width(w), height(h), depth(d)
{
}

AABB::AABB(glm::vec3 p, double w, double h, double d) : 
               position(p), width(w), height(h), depth(d)
{
}

AABB::AABB(glm::vec3 p1, glm::vec3 p2) 
{
    double minx = fmin(p1.x, p2.x);
    double miny = fmin(p1.y, p2.y);
    double minz = fmin(p1.z, p2.z);
    double maxx = fmax(p1.x, p2.x);
    double maxy = fmax(p1.y, p2.y);
    double maxz = fmax(p1.z, p2.z);

    position = glm::vec3(minx, miny, minz);
    width = maxx - minx;
    height = maxy - miny;
    depth = maxz - minz;
}

AABB::AABB(std::vector<glm::vec3> &points) {
    double minx = points[0].x;
    double miny = points[0].y;
    double minz = points[0].z;
    double maxx = points[0].x;
    double maxy = points[0].y;
    double maxz = points[0].z;

    glm::vec3 p;
    for (unsigned int i = 0; i < points.size(); i++) {
        p = points[i];
        minx = fmin(p.x, minx);
        miny = fmin(p.y, miny);
        minz = fmin(p.z, minz);
        maxx = fmax(p.x, maxx);
        maxy = fmax(p.y, maxy);
        maxz = fmax(p.z, maxz);
    }

    position = glm::vec3(minx, miny, minz);
    width = maxx - minx;
    height = maxy - miny;
    depth = maxz - minz;
}

AABB::AABB(Triangle t, std::vector<glm::vec3> &vertices) {
    glm::vec3 points[3] = { vertices[t.tri[0]],
                            vertices[t.tri[1]],
                            vertices[t.tri[2]] };

    double minx = points[0].x;
    double miny = points[0].y;
    double minz = points[0].z;
    double maxx = points[0].x;
    double maxy = points[0].y;
    double maxz = points[0].z;

    glm::vec3 p;
    for (int i = 0; i < 3; i++) {
        p = points[i];
        minx = fmin(p.x, minx);
        miny = fmin(p.y, miny);
        minz = fmin(p.z, minz);
        maxx = fmax(p.x, maxx);
        maxy = fmax(p.y, maxy);
        maxz = fmax(p.z, maxz);
    }

    position = glm::vec3(minx, miny, minz);
    width = maxx - minx;
    height = maxy - miny;
    depth = maxz - minz;
}

AABB::AABB(GridIndex g, double dx) {
    position = glm::vec3(g.i*dx, g.j*dx, g.k*dx);
    width = height = depth = dx;
}

AABB::~AABB()
{
}

void AABB::expand(double v) {
    double h = 0.5 * v;
    position -= glm::vec3(h, h, h);
    width += v;
    height += v;
    depth += v;
}

bool AABB::isPointInside(glm::vec3 p) {
    return p.x >= position.x && p.y >= position.y && p.z >= position.z &&
           p.x < position.x + width && p.y < position.y + height && p.z < position.z + depth;
}

GridIndex AABB::_positionToGridIndex(glm::vec3 p, double dx) {
    double invdx = 1.0 / dx;
    return GridIndex((int)floor(p.x*invdx),
                     (int)floor(p.y*invdx),
                     (int)floor(p.z*invdx));
}

void AABB::getOverlappingGridCells(double dx, std::vector<GridIndex> &cells) {
    glm::vec3 v = position + glm::vec3(width, height, depth);

    GridIndex g1 = _positionToGridIndex(position, dx);
    GridIndex g2 = _positionToGridIndex(v, dx);

    for (int k = g1.k; k <= g2.k; k++) {
        for (int j = g1.j; j <= g2.j; j++) {
            for (int i = g1.i; i <= g2.i; i++) {
                cells.push_back(GridIndex(i, j, k));
            }
        }
    }
}

bool AABB::isLineIntersecting(glm::vec3 p1, glm::vec3 p2) {

    glm::vec3 min = position;
    glm::vec3 max = position + glm::vec3(width, height, depth);

    glm::vec3 d = (p2 - p1) * 0.5f;
    glm::vec3 e = (max - min) * 0.5f;
    glm::vec3 c = p1 + d - (min + max) * 0.5f;
    glm::vec3 ad = glm::vec3(fabs(d.x), fabs(d.y), fabs(d.z));

    if (fabs(c.x) > e.x + ad.x) {
        return false;
    }
    if (fabs(c.y) > e.y + ad.y) {
        return false;
    }
    if (fabs(c.z) > e.z + ad.z) {
        return false;
    }


    double eps = 10e-9;
    if (fabs(d.y * c.z - d.z * c.y) > e.y * ad.z + e.z * ad.y + eps) {
        return false;
    }
    if (fabs(d.z * c.x - d.x * c.z) > e.z * ad.x + e.x * ad.z + eps) {
        return false;
    }
    if (fabs(d.x * c.y - d.y * c.x) > e.x * ad.y + e.y * ad.x + eps) {
        return false;
    }

    return true;
}

void AABB::_findminmax(double x0, double x1, double x2, double *min, double *max) {
    *min = *max = x0;

    if (x1 < *min) { *min = x1; }
    if (x1 > *max) { *max = x1; }
    if (x2 < *min) { *min = x2; }
    if (x2 > *max) { *max = x2; }
}

bool AABB::_planeBoxOverlap(glm::vec3 normal, glm::vec3 vert) {
    double minx, miny, minz, maxx, maxy, maxz;
    double vx, vy, vz;

    vx = vert.x;
    vy = vert.y;
    vz = vert.z;

    if (normal.x > 0.0) {
        minx = -0.5*width - vx;
        maxx = 0.5*width - vx;
    } else {
        minx = 0.5*width - vx;
        maxx = -0.5*width - vx;
    }

    if (normal.y > 0.0) {
        miny = -0.5*height - vy;
        maxy = 0.5*height - vy;
    }
    else {
        miny = 0.5*height - vy;
        maxy = -0.5*height - vy;
    }

    if (normal.z > 0.0) {
        minz = -0.5*depth - vz;
        maxz = 0.5*depth - vz;
    }
    else {
        minz = 0.5*depth - vz;
        maxz = -0.5*depth - vz;
    }

    glm::vec3 vmin = glm::vec3(minx, miny, minz);
    glm::vec3 vmax = glm::vec3(maxx, maxy, maxz);

    if (glm::dot(normal, vmin) > 0.0) {
        return false;
    }
    if (glm::dot(normal, vmax) >= 0.0) {
        return true;
    }

    return false;
}

bool AABB::_axisTestX01(glm::vec3 v0, glm::vec3 v2, 
                        double a, double b, double fa, double fb) {
    double p0 = (float)a*v0.y - (float)b*v0.z;
    double p2 = (float)a*v2.y - (float)b*v2.z;

    double min, max;
    if (p0 < p2) {
        min = p0;
        max = p2;
    }
    else {
        min = p2;
        max = p0;
    }

    double rad = 0.5*height*fa + 0.5*depth*fb;

    if (min > rad || max < -rad) {
        return false;
    }

    return true;
}

bool AABB::_axisTestX2(glm::vec3 v0, glm::vec3 v1, 
                       double a, double b, double fa, double fb) {
    double p0 = (float)a*v0.y - (float)b*v0.z;
    double p1 = (float)a*v1.y - (float)b*v1.z;

    double min, max;
    if (p0 < p1) {
        min = p0;
        max = p1;
    }
    else {
        min = p1;
        max = p0;
    }

    double rad = 0.5*height*fa + 0.5*depth*fb;

    if (min > rad || max < -rad) {
        return false;
    }

    return true;
}

bool AABB::_axisTestY02(glm::vec3 v0, glm::vec3 v2, 
                        double a, double b, double fa, double fb) {
    double p0 = (float)-a*v0.x + (float)b*v0.z;
    double p2 = (float)-a*v2.x + (float)b*v2.z;

    double min, max;
    if (p0 < p2) {
        min = p0;
        max = p2;
    }
    else {
        min = p2;
        max = p0;
    }

    double rad = 0.5*width*fa + 0.5*depth*fb;

    if (min > rad || max < -rad) {
        return false;
    }

    return true;
}

bool AABB::_axisTestY1(glm::vec3 v0, glm::vec3 v1,
                       double a, double b, double fa, double fb) {
    double p0 = (float)-a*v0.x + (float)b*v0.z;
    double p1 = (float)-a*v1.x + (float)b*v1.z;

    double min, max;
    if (p0 < p1) {
        min = p0;
        max = p1;
    }
    else {
        min = p1;
        max = p0;
    }

    double rad = 0.5*width*fa + 0.5*depth*fb;

    if (min > rad || max < -rad) {
        return false;
    }

    return true;
}

bool AABB::_axisTestZ12(glm::vec3 v1, glm::vec3 v2,
                       double a, double b, double fa, double fb) {
    double p1 = (float)a*v1.x - (float)b*v1.y;
    double p2 = (float)a*v2.x - (float)b*v2.y;

    double min, max;
    if (p2 < p1) {
        min = p2;
        max = p1;
    }
    else {
        min = p1;
        max = p2;
    }

    double rad = 0.5*width*fa + 0.5*height*fb;

    if (min > rad || max < -rad) {
        return false;
    }

    return true;
}

bool AABB::_axisTestZ0(glm::vec3 v0, glm::vec3 v1,
                       double a, double b, double fa, double fb) {
    double p0 = (float)a*v0.x - (float)b*v0.y;
    double p1 = (float)a*v1.x - (float)b*v1.y;

    double min, max;
    if (p0 < p1) {
        min = p0;
        max = p1;
    }
    else {
        min = p1;
        max = p0;
    }

    double rad = 0.5*width*fa + 0.5*height*fb;

    if (min > rad || max < -rad) {
        return false;
    }

    return true;
}

// adapted from the method described in this paper: 
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
bool AABB::isOverlappingTriangle(Triangle t, std::vector<glm::vec3> &vertices) {
    glm::vec3 tv0 = vertices[t.tri[0]];
    glm::vec3 tv1 = vertices[t.tri[1]];
    glm::vec3 tv2 = vertices[t.tri[2]];

    if (isPointInside(tv0) || isPointInside(tv1) || isPointInside(tv2)) {
        return true;
    }

    if (isLineIntersecting(tv0, tv1) || 
        isLineIntersecting(tv1, tv2) || 
        isLineIntersecting(tv2, tv0)) {
        return true;
    }

    glm::vec3 boxcenter = position + (float)0.5 * glm::vec3(width, height, depth);

    glm::vec3 v0 = tv0 - boxcenter;
    glm::vec3 v1 = tv1 - boxcenter;
    glm::vec3 v2 = tv2 - boxcenter;

    glm::vec3 e0 = v1 - v0;
    glm::vec3 e1 = v2 - v1;
    glm::vec3 e2 = v0 - v2;

    double fex = fabs(e0.x);
    double fey = fabs(e0.y);
    double fez = fabs(e0.z);

    if (!_axisTestX01(v0, v2, e0.z, e0.y, fez, fey)) { return false; }
    if (!_axisTestY02(v0, v2, e0.z, e0.x, fez, fex)) { return false; }
    if (!_axisTestZ12(v1, v2, e0.y, e0.x, fey, fex)) { return false; }

    fex = fabs(e1.x);
    fey = fabs(e1.y);
    fez = fabs(e1.z);

    if (!_axisTestX01(v0, v2, e1.z, e1.y, fez, fey)) { return false; }
    if (!_axisTestY02(v0, v2, e1.z, e1.x, fez, fex)) { return false; }
    if (!_axisTestZ0(v0, v1, e1.y, e1.x, fey, fex)) { return false; }

    fex = fabs(e2.x);
    fey = fabs(e2.y);
    fez = fabs(e2.z);

    if (!_axisTestX2(v0, v1, e2.z, e2.y, fez, fey)) { return false; }
    if (!_axisTestY1(v0, v1, e2.z, e2.x, fez, fex)) { return false; }
    if (!_axisTestZ12(v1, v2, e2.y, e2.x, fey, fex)) { return false; }

    double min, max;
    _findminmax(v0.x, v1.x, v2.x, &min, &max);
    if (min > 0.5*width || max < -0.5*width) {
        return false;
    }

    _findminmax(v0.y, v1.y, v2.y, &min, &max);
    if (min > 0.5*height || max < -0.5*height) {
        return false;
    }

    _findminmax(v0.z, v1.z, v2.z, &min, &max);
    if (min > 0.5*depth || max < -0.5*depth) {
        return false;
    }

    glm::vec3 normal = glm::cross(e0, e1);
    if (!_planeBoxOverlap(normal, v0)) {
        return false;
    }

    return true;
}


