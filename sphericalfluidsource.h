#pragma once
#include "fluidsource.h"

#include <vector>

#include "glm/glm.hpp"
#include "aabb.h"

class SphericalFluidSource : public FluidSource
{
public:
    SphericalFluidSource();
    SphericalFluidSource(glm::vec3 pos, double r);
    SphericalFluidSource(glm::vec3 pos, double r, glm::vec3 velocity);
    ~SphericalFluidSource();

    void setRadius(double r);
    double getRadius();

    virtual std::vector<GridIndex> getNewFluidCells(Array3d<int> &materialGrid,
                                                    double dx);

private:

    void _getOverlappingGridIndices(std::vector<GridIndex> &storage,
                                    int i, int j, int k, double dx);

    double _radius = 0.0;

};

