#include "implicitfield.h"


ImplicitField::ImplicitField()
{
}

ImplicitField::ImplicitField(int w, int h, int d, double cell_size) :
                             i_width(w), j_height(h), k_depth(d), dx(cell_size),
                             width((double)w*cell_size), 
                             height((double)h*cell_size), 
                             depth((double)d*cell_size)
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

void ImplicitField::_positionToGridIndex(glm::vec3 p, int *i, int *j, int *k) {
    double invdx = 1.0 / dx;
    *i = (int)floor(p.x*invdx);
    *j = (int)floor(p.y*invdx);
    *k = (int)floor(p.z*invdx);
}

bool ImplicitField::_isPointNearSolid(glm::vec3 p) {
    double eps = 10e-9;

    glm::vec3 x = glm::vec3(eps, 0.0, 0.0);
    glm::vec3 y = glm::vec3(0.0, eps, 0.0);
    glm::vec3 z = glm::vec3(0.0, 0.0, eps);

    int i, j, k;
    _positionToGridIndex(p, &i, &j, &k);
    if (materialGrid(i, j, k) == 2) {
        return true;
    }

    glm::vec3 points[26];
    points[0] = p - x;
    points[1] = p + x;
    points[2] = p - y;
    points[3] = p + y;
    points[4] = p - z;
    points[5] = p + z;
    points[6] = p - x - y;
    points[7] = p - x + y;
    points[8] = p + x - y;
    points[9] = p + x + y;
    points[10] = p - x - z;
    points[11] = p - x + z;
    points[12] = p + x - z;
    points[13] = p + x + z;
    points[14] = p - y - z;
    points[15] = p - y + z;
    points[16] = p + y - z;
    points[17] = p + y + z;
    points[18] = p - x - y - z;
    points[19] = p - x - y + z;
    points[20] = p - x + y - z;
    points[21] = p - x + y + z;
    points[22] = p + x - y - z;
    points[23] = p + x - y + z;
    points[24] = p + x + y - z;
    points[25] = p + x + y + z;
    
    for (int idx = 0; idx < 26; idx++) {
        _positionToGridIndex(points[idx], &i, &j, &k);
        if (materialGrid(i, j, k) == 2) {
            return true;
        }
    }

    return false;
}

double ImplicitField::getFieldValue(glm::vec3 p) {
    double eps = 10e-6;
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

    if (isMaterialGridSet && sum > surfaceThreshold && _isPointNearSolid(p)) {
        return surfaceThreshold - eps;
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