#pragma once
#include "surfacefield.h"

#include "spatialgrid.h"

struct ImplicitPointData {
    glm::vec3 position = glm::vec3(0.0, 0.0, 0.0);
    double radius = 0.0;
};


class ImplicitSurfaceField : public SurfaceField
{
public:
    ImplicitSurfaceField();
    ImplicitSurfaceField(int i_width, int j_height, int k_depth, double cell_size);
    ~ImplicitSurfaceField();

    void addPoint(double x, double y, double z, double r) { addPoint(glm::vec3(x, y, z), r); };
    void addPoint(glm::vec3 p, double r);
    void addCuboid(double x, double y, double z, double w, double h, double d) {
        addCuboid(glm::vec3(x, y, z), w, h, d);
    }
    void addCuboid(glm::vec3 p, double width, double height, double depth);

    double getSurfaceThreshold() { return _surfaceThreshold; }
    double getRicciBlend() { return _ricciBlend; }
    int getNumPoints() { return (int)_points.size(); }
    int getNumCuboids() { return (int)_cuboids.size(); }

    void setRicciBlend(double k) { assert(k > 0.0); _ricciBlend = k; }

    std::vector<ImplicitPointData> getImplicitPointData();

    virtual void clear();
    virtual double getFieldValue(glm::vec3 p);

private: 
    // cuboid has a field value of (surfaceThreshold + epsilon) within
    // its volume, 0 outside
    struct Cuboid {
        glm::vec3 position;
        double width, height, depth;

        Cuboid() : position(glm::vec3(0.0, 0.0, 0.0)),
                   width(0.0), height(0.0), depth(0.0) {}

        Cuboid(glm::vec3 p, double w, double h, double d) :
               position(p), width(w), height(h), depth(d) {}
    };

    bool _isPointInsideCuboid(glm::vec3 p, Cuboid c);

    std::vector<ImplicitPointPrimitive> _points;
    std::vector<Cuboid> _cuboids;

    double _surfaceThreshold = 0.5;
    double _ricciBlend = 1.0;

    int _gridi, _gridj, _gridk;
    SpatialGrid<ImplicitPointPrimitive> _pointGrid;
};

