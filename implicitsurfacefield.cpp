#include "implicitsurfacefield.h"


ImplicitSurfaceField::ImplicitSurfaceField()
{
}

ImplicitSurfaceField::ImplicitSurfaceField(int i, int j, int k, double dx) :
                                SurfaceField(i, j, k, dx),
                                _gridi(i), _gridj(j), _gridk(k),
                                _pointGrid(_gridi / _pointGridCellSize,
                                           _gridj / _pointGridCellSize, 
                                           _gridk / _pointGridCellSize, 
                                           dx*_pointGridCellSize)
{
    setSurfaceThreshold(_surfaceThreshold);
}

ImplicitSurfaceField::~ImplicitSurfaceField()
{
}

void ImplicitSurfaceField::addPoint(glm::vec3 p, double r) {
    ImplicitPointPrimitive point = ImplicitPointPrimitive(p, r);
    _points.push_back(point);
    _pointGrid.insert(point, p, r);
}

void ImplicitSurfaceField::addCuboid(glm::vec3 p, double w, double h, double d) {
    _cuboids.push_back(Cuboid(p, w, h, d));
}

void ImplicitSurfaceField::clear() {
    _points.clear();
    _cuboids.clear();
    _pointGrid.clear();
}

double ImplicitSurfaceField::getFieldValue(glm::vec3 p) {
    double eps = 10e-6;
    bool isBlending = fabs(_ricciBlend - 1.0) > eps;
    double sum = 0.0;

    std::vector<ImplicitPointPrimitive> points;
    _pointGrid.query(p, points);

    ImplicitPointPrimitive pi;
    for (unsigned int i = 0; i < points.size(); i++) {
        pi = points[i];
        if (isBlending) {
            sum += powl(pi.getFieldValue(p), _ricciBlend);
        } else {
            sum += pi.getFieldValue(p);
        }
    }

    Cuboid ci;
    for (unsigned int i = 0; i < _cuboids.size(); i++) {
        ci = _cuboids[i];
        if (_isPointInsideCuboid(p, ci)) {
            if (isBlending) {
                sum += powl(_surfaceThreshold + eps, _ricciBlend);
            }
            else {
                sum += _surfaceThreshold + eps;
            }
        }
    }

    if (isBlending && sum > 0.0) {
        sum = powl(sum, 1.0 / _ricciBlend);
    }

    if (sum < 0.0) {
        sum = 0.0;
    } else if (sum > 1.0) {
        sum = 1.0;
    }

    if (isMaterialGridSet && sum > _surfaceThreshold && _isPointNearSolid(p)) {
        return _surfaceThreshold - eps;
    }

    return sum;
}

std::vector<ImplicitPointData> ImplicitSurfaceField::getImplicitPointData() {
    std::vector<ImplicitPointData> data;
    for (unsigned int i = 0; i < _points.size(); i++) {
        ImplicitPointData d;
        d.position = _points[i].getPosition();
        d.radius = _points[i].getRadius();
        data.push_back(d);
    }

    return data;
}

bool ImplicitSurfaceField::_isPointInsideCuboid(glm::vec3 p, Cuboid c) {
    return p.x >= c.position.x && p.x < c.position.x + c.width &&
           p.y >= c.position.y && p.y < c.position.y + c.height &&
           p.z >= c.position.z && p.z < c.position.z + c.depth;
}