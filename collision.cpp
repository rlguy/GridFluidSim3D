#include "collision.h"


// method adapted from:
// http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
bool Collision::rayIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                      glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, 
                                      glm::vec3 *collision) {
    glm::vec3 e1, e2, h, s, q;
    float a, f, u, v;

    e1 = v1 - v0;
    e2 = v2 - v0;
    h = glm::cross(e1, h);
    a = glm::dot(e1, h);

    double eps = 10e-9;
    if (fabs(a) < eps) {
        return false;
    }

    f = 1 / a;
    s = p - v0;
    u = f * glm::dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;
    }

    q = glm::cross(s, e1);
    v = f * glm::dot(dir, q);

    if (v < 0.0 || u + v > 1.0) {
        return false;
    }

    // at this stage we can compute t to find out where
    // the intersection point is on the line
    double t = f * glm::dot(e2, q);

    if (t > 0) {
        *collision = p + (float)t*dir;
        return true;
    } 
    else {
        return false;
    }
}

// method adapted from:
// http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
bool Collision::lineIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                       glm::vec3 v0, glm::vec3 v1, glm::vec3 v2,
    glm::vec3 *collision) {
    glm::vec3 e1, e2, h, s, q;
    float a, f, u, v;

    e1 = v1 - v0;
    e2 = v2 - v0;
    h = glm::cross(e1, h);
    a = glm::dot(e1, h);

    double eps = 10e-9;
    if (fabs(a) < eps) {
        return false;
    }

    f = 1 / a;
    s = p - v0;
    u = f * glm::dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;
    }

    q = glm::cross(s, e1);
    v = f * glm::dot(dir, q);

    if (v < 0.0 || u + v > 1.0) {
        return false;
    }

    // at this stage we can compute t to find out where
    // the intersection point is on the line
    double t = f * glm::dot(e2, q);
    *collision = p + (float)t*dir;

    return true;
}

bool Collision::rayIntersectsPlane(glm::vec3 p0, glm::vec3 dir, 
                                    glm::vec3 planePoint, glm::vec3 planeNormal, 
                                    glm::vec3 *collision) {

    // perpendicular or p0 is on the plane
    double eps = 10e-9;
    double dot = glm::dot(dir, planeNormal);
    if (fabs(dot) < eps) {
        return false;
    }

    double d = glm::dot((planePoint - p0), planeNormal) / dot;
    if (d > 0) {
        *collision = p0 + (float)d*dir;
        return true;
    }

    return false;
}

bool Collision::lineIntersectsPlane(glm::vec3 p0, glm::vec3 dir, 
                                    glm::vec3 planePoint, glm::vec3 planeNormal, 
                                    glm::vec3 *collision) {

    // perpendicular or p0 is on the plane
    double eps = 10e-9;
    double dot = glm::dot(dir, planeNormal);
    if (fabs(dot) < eps) {
        return false;
    }

    double d = glm::dot((planePoint - p0), planeNormal) / dot;

    *collision = p0 + (float)d*dir;

    return true;
}


// method adapted from:
// http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
glm::vec3 Collision::findClosestPointOnTriangle(glm::vec3 p0, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2) {
    glm::vec3 edge0 = v1 - v0;
    glm::vec3 edge1 = v2 - v0;
    glm::vec3 pv = v0 - p0;

    double a = glm::dot(edge0, edge0);
    double b = glm::dot(edge0, edge1);
    double c = glm::dot(edge1, edge1);
    double d = glm::dot(edge0, pv);
    double e = glm::dot(edge1, pv);

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e;

    if (s + t < det)
    {
        if (s < 0.0)
        {
            if (t < 0.0)
            {
                if (d < 0.0)
                {
                    s = _clamp(-d / a, 0.0, 1.0);
                    t = 0.0;
                }
                else
                {
                    s = 0.0;
                    t = _clamp(-e / c, 0.0, 1.0);
                }
            }
            else
            {
                s = 0.0;
                t = _clamp(-e / c, 0.0, 1.0);
            }
        }
        else if (t < 0.0)
        {
            s = _clamp(-d / a, 0.0, 1.0);
            t = 0.0;
        }
        else
        {
            double invDet = 1.0 / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if (s < 0.0)
        {
            double tmp0 = b + d;
            double tmp1 = c + e;
            if (tmp1 > tmp0)
            {
                double numer = tmp1 - tmp0;
                double denom = a - 2 * b + c;
                s = _clamp(numer / denom, 0.0, 1.0);
                t = 1 - s;
            }
            else
            {
                t = _clamp(-e / c, 0.0, 1.0);
                s = 0.0;
            }
        }
        else if (t < 0.0)
        {
            if (a + d > b + e)
            {
                double numer = c + e - b - d;
                double denom = a - 2 * b + c;
                s = _clamp(numer / denom, 0.0, 1.0);
                t = 1 - s;
            }
            else
            {
                s = _clamp(-e / c, 0.0, 1.0);
                t = 0.0;
            }
        }
        else
        {
            double numer = c + e - b - d;
            double denom = a - 2 * b + c;
            s = _clamp(numer / denom, 0.0, 1.0);
            t = 1.0 - s;
        }
    }

    return v0 + (float)s * edge0 + (float)t * edge1;
}

bool Collision::rayIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                      glm::vec3 v0, glm::vec3 v1, glm::vec3 v2) {
    glm::vec3 i;
    return rayIntersectsTriangle(p, dir, v0, v1, v2, &i);
}

bool Collision::lineIntersectsTriangle(glm::vec3 p, glm::vec3 dir,
                                       glm::vec3 v0, glm::vec3 v1, glm::vec3 v2) {
    glm::vec3 i;
    return lineIntersectsTriangle(p, dir, v0, v1, v2, &i);
}

bool Collision::rayIntersectsPlane(glm::vec3 p0, glm::vec3 dir,
                                   glm::vec3 planePoint, glm::vec3 planeNormal) {
    glm::vec3 p;
    return rayIntersectsPlane(p0, dir, planePoint, planeNormal, &p);
}

bool Collision::lineIntersectsPlane(glm::vec3 p0, glm::vec3 dir,
                                    glm::vec3 planePoint, glm::vec3 planeNormal) {
    glm::vec3 p;
    return lineIntersectsPlane(p0, dir, planePoint, planeNormal, &p);
}

double Collision::_clamp(double v, double min, double max) {
    if (v < min) { v = min; }
    if (v > max) { v = max; }

    return v;
}