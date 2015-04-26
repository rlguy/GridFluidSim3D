#include "implicitfield.h"


ImplicitField::ImplicitField()
{
}

ImplicitField::ImplicitField(double w, double h, double d) :
                             width(w), height(h), depth(d)
{
}

ImplicitField::~ImplicitField()
{
}

void ImplicitField::addPoint(glm::vec3 p, double r) {
    points.push_back(ImplicitPointPrimitive(p, r));
}

void ImplicitField::addCuboid(glm::vec3 p, double w, double h, double d) {
    cuboids.push_back(Cuboid(p, w, h, d));
}

void ImplicitField::clear() {
    points.clear();
}

double ImplicitField::getFieldValue(glm::vec3 p) {
    double eps = 10e-3;
    bool isBlending = fabs(ricciBlend - 1.0) > eps;
    double sum = 0.0;

    ImplicitPointPrimitive pi;
    for (int i = 0; i < (int)points.size(); i++) {
        pi = points[i];
        if (isBlending) {
            sum += powl(pi.getFieldValue(p), ricciBlend);
        } else {
            sum += pi.getFieldValue(p);
        }
    }

    Cuboid ci;
    for (int i = 0; i < (int)cuboids.size(); i++) {
        ci = cuboids[i];
        if (_isPointInsideCuboid(p, ci)) {
            if (isBlending) {
                sum += powl(surfaceThreshold + eps, ricciBlend);
            }
            else {
                sum += surfaceThreshold + eps;
            }
        }
    }

    if (isBlending && sum > 0.0) {
        sum = powl(sum, 1.0 / ricciBlend);
    }

    if (sum < 0.0) {
        sum = 0.0;
    } else if (sum > 1.0) {
        sum = 1.0;
    }

    return sum;
}

bool ImplicitField::isInside(glm::vec3 p) {
    return getFieldValue(p) > surfaceThreshold;
}

bool ImplicitField::isOutside(glm::vec3 p) {
    return getFieldValue(p) <= surfaceThreshold;
}

std::vector<ImplicitPointData> ImplicitField::getImplicitPointData() {
    std::vector<ImplicitPointData> data;
    for (int i = 0; i < (int)points.size(); i++) {
        ImplicitPointData d;
        d.position = points[i].getPosition();
        d.radius = points[i].getRadius();
        data.push_back(d);
    }

    return data;
}

bool ImplicitField::_isPointInsideCuboid(glm::vec3 p, Cuboid c) {
    return p.x >= c.position.x && p.x < c.position.x + c.width &&
           p.y >= c.position.y && p.y < c.position.y + c.height &&
           p.z >= c.position.z && p.z < c.position.z + c.depth;
}