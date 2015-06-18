#include "implicitsurfacefield.h"


ImplicitSurfaceField::ImplicitSurfaceField()
{
}

ImplicitSurfaceField::ImplicitSurfaceField(int w, int h, int d, double dx) :
                                SurfaceField(w, h, d, dx),
                                surfaceThreshold(0.5)
{
    gridi = ceil(w / dx);
    gridj = ceil(h / dx);
    gridk = ceil(d / dx);
    pointGrid = SpatialGrid<ImplicitPointPrimitive>(gridi, gridj, gridk, dx);
}

ImplicitSurfaceField::~ImplicitSurfaceField()
{
}

void ImplicitSurfaceField::addPoint(glm::vec3 p, double r) {
    ImplicitPointPrimitive point = ImplicitPointPrimitive(p, r);
    points.push_back(point);
    pointGrid.insert(point, p, r);
}

void ImplicitSurfaceField::addCuboid(glm::vec3 p, double w, double h, double d) {
    cuboids.push_back(Cuboid(p, w, h, d));
}

void ImplicitSurfaceField::clear() {
    points.clear();
    cuboids.clear();
    pointGrid.clear();
}

double ImplicitSurfaceField::getFieldValue(glm::vec3 p) {
    double eps = 10e-6;
    bool isBlending = fabs(ricciBlend - 1.0) > eps;
    double sum = 0.0;

    std::vector<ImplicitPointPrimitive> points;
    pointGrid.query(p, points);

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

std::vector<ImplicitPointData> ImplicitSurfaceField::getImplicitPointData() {
    std::vector<ImplicitPointData> data;
    for (int i = 0; i < (int)points.size(); i++) {
        ImplicitPointData d;
        d.position = points[i].getPosition();
        d.radius = points[i].getRadius();
        data.push_back(d);
    }

    return data;
}

bool ImplicitSurfaceField::_isPointInsideCuboid(glm::vec3 p, Cuboid c) {
    return p.x >= c.position.x && p.x < c.position.x + c.width &&
           p.y >= c.position.y && p.y < c.position.y + c.height &&
           p.z >= c.position.z && p.z < c.position.z + c.depth;
}