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
#include "collision.h"

// method adapted from:
// http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
bool Collision::rayIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                      vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2, 
                                      vmath::vec3 *collision, double *iu, double *iv) {
    vmath::vec3 e1, e2, h, s, q;
    float a, f, u, v;

    e1 = v1 - v0;
    e2 = v2 - v0;
    h = vmath::cross(dir, e2);
    a = vmath::dot(e1, h);

    double eps = 10e-9;
    if (fabs(a) < eps) {
        return false;
    }

    f = 1 / a;
    s = p - v0;
    u = f * vmath::dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;
    }

    q = vmath::cross(s, e1);
    v = f * vmath::dot(dir, q);

    if (v < 0.0 || u + v > 1.0) {
        return false;
    }

    // at this stage we can compute t to find out where
    // the intersection point is on the line
    double t = f * vmath::dot(e2, q);

    if (t > 0) {
        *collision = p + (float)t*dir;
        *iu = u;
        *iv = v;
        return true;
    } 
    else {
        return false;
    }
}

// method adapted from:
// http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
bool Collision::lineIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                       vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2,
                                       vmath::vec3 *collision, double *iu, double *iv) {
    vmath::vec3 e1, e2, h, s, q;
    float a, f, u, v;

    e1 = v1 - v0;
    e2 = v2 - v0;
    h = vmath::cross(dir, e2);
    a = vmath::dot(e1, h);

    double eps = 10e-9;
    if (fabs(a) < eps) {
        return false;
    }

    f = 1 / a;
    s = p - v0;
    u = f * vmath::dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;
    }

    q = vmath::cross(s, e1);
    v = f * vmath::dot(dir, q);

    if (v < 0.0 || u + v > 1.0) {
        return false;
    }

    // at this stage we can compute t to find out where
    // the intersection point is on the line
    double t = f * vmath::dot(e2, q);
    *collision = p + (float)t*dir;
    *iu = u;
    *iv = v;

    return true;
}

bool Collision::rayIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir, 
                                    vmath::vec3 planePoint, vmath::vec3 planeNormal, 
                                    vmath::vec3 *collision) {

    // perpendicular or p0 is on the plane
    double eps = 10e-9;
    double dot = vmath::dot(dir, planeNormal);
    if (fabs(dot) < eps) {
        return false;
    }

    double d = vmath::dot((planePoint - p0), planeNormal) / dot;
    if (d > 0) {
        *collision = p0 + (float)d*dir;
        return true;
    }

    return false;
}

bool Collision::lineIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir, 
                                    vmath::vec3 planePoint, vmath::vec3 planeNormal, 
                                    vmath::vec3 *collision) {

    // perpendicular or p0 is on the plane
    double eps = 10e-9;
    double dot = vmath::dot(dir, planeNormal);
    if (fabs(dot) < eps) {
        return false;
    }

    double d = vmath::dot((planePoint - p0), planeNormal) / dot;

    *collision = p0 + (float)d*dir;

    return true;
}


// method adapted from:
// http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
vmath::vec3 Collision::findClosestPointOnTriangle(vmath::vec3 p0, vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2) {
    vmath::vec3 edge0 = v1 - v0;
    vmath::vec3 edge1 = v2 - v0;
    vmath::vec3 pv = v0 - p0;

    double a = vmath::dot(edge0, edge0);
    double b = vmath::dot(edge0, edge1);
    double c = vmath::dot(edge1, edge1);
    double d = vmath::dot(edge0, pv);
    double e = vmath::dot(edge1, pv);

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e;

    if (s + t < det) {
        if (s < 0.0) {
            if (t < 0.0) {
                if (d < 0.0) {
                    s = _clamp(-d / a, 0.0, 1.0);
                    t = 0.0;
                }
                else {
                    s = 0.0;
                    t = _clamp(-e / c, 0.0, 1.0);
                }
            } else {
                s = 0.0;
                t = _clamp(-e / c, 0.0, 1.0);
            }
        } else if (t < 0.0) {
            s = _clamp(-d / a, 0.0, 1.0);
            t = 0.0;
        } else {
            double invDet = 1.0 / det;
            s *= invDet;
            t *= invDet;
        }
    } else {
        if (s < 0.0) {
            double tmp0 = b + d;
            double tmp1 = c + e;
            if (tmp1 > tmp0) {
                double numer = tmp1 - tmp0;
                double denom = a - 2 * b + c;
                s = _clamp(numer / denom, 0.0, 1.0);
                t = 1 - s;
            } else {
                t = _clamp(-e / c, 0.0, 1.0);
                s = 0.0;
            }
        } else if (t < 0.0) {
            if (a + d > b + e) {
                double numer = c + e - b - d;
                double denom = a - 2 * b + c;
                s = _clamp(numer / denom, 0.0, 1.0);
                t = 1 - s;
            } else {
                s = _clamp(-e / c, 0.0, 1.0);
                t = 0.0;
            }
        } else {
            double numer = c + e - b - d;
            double denom = a - 2 * b + c;
            s = _clamp(numer / denom, 0.0, 1.0);
            t = 1.0 - s;
        }
    }

    return v0 + (float)s * edge0 + (float)t * edge1;
}

bool Collision::rayIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                      vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2) {
    vmath::vec3 i;
    double u, v;
    return rayIntersectsTriangle(p, dir, v0, v1, v2, &i, &u, &v);
}

bool Collision::rayIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                      vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2, vmath::vec3 *i) {
    double u, v;
    return rayIntersectsTriangle(p, dir, v0, v1, v2, i, &u, &v);
}

bool Collision::lineIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                       vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2) {
    vmath::vec3 i;
    double u, v;
    return lineIntersectsTriangle(p, dir, v0, v1, v2, &i, &u, &v);
}

bool Collision::lineIntersectsTriangle(vmath::vec3 p, vmath::vec3 dir,
                                       vmath::vec3 v0, vmath::vec3 v1, vmath::vec3 v2, vmath::vec3 *i) {
    double u, v;
    return lineIntersectsTriangle(p, dir, v0, v1, v2, i, &u, &v);
}

bool Collision::rayIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir,
                                   vmath::vec3 planePoint, vmath::vec3 planeNormal) {
    vmath::vec3 p;
    return rayIntersectsPlane(p0, dir, planePoint, planeNormal, &p);
}

bool Collision::lineIntersectsPlane(vmath::vec3 p0, vmath::vec3 dir,
                                    vmath::vec3 planePoint, vmath::vec3 planeNormal) {
    vmath::vec3 p;
    return lineIntersectsPlane(p0, dir, planePoint, planeNormal, &p);
}

double Collision::_clamp(double v, double min, double max) {
    if (v < min) { v = min; }
    if (v > max) { v = max; }

    return v;
}

vmath::vec3 Collision::getTriangleCentroid(vmath::vec3 p0, vmath::vec3 p1, vmath::vec3 p2) {
    return (1.0/3.0) * (p0 + p1 + p2);
}

vmath::vec3 Collision::getTriangleNormal(vmath::vec3 p0, vmath::vec3 p1, vmath::vec3 p2) {
    vmath::vec3 v1 = p1 - p0;
    vmath::vec3 v2 = p2 - p0;
    vmath::vec3 v3 = p2 - p1;

    double eps = 1e-9;
    if (fabs(v1.x) < eps && fabs(v1.y) < eps && fabs(v1.z) < eps) {
        return vmath::vec3();
    }
    if (fabs(v2.x) < eps && fabs(v2.y) < eps && fabs(v2.z) < eps) {
        return vmath::vec3();
    }
    if (fabs(v3.x) < eps && fabs(v3.y) < eps && fabs(v3.z) < eps) {
        return vmath::vec3();
    }

    return vmath::normalize(vmath::cross(v1, v2));
}