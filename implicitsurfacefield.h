#pragma once
#include "surfacefield.h"

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

    int _pointGridCellSize = 2; // cell size for point grid = _pointGridCellSize*dx
    int _gridi, _gridj, _gridk;
    SpatialGrid<ImplicitPointPrimitive> _pointGrid;
};

