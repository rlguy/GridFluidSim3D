#pragma once
#include "fluidsource.h"

#include <vector>

#include "glm/glm.hpp"
#include "aabb.h"

class CuboidFluidSource : public FluidSource
{
public:
    CuboidFluidSource();
    CuboidFluidSource(glm::vec3 position, double w, double h, double d);
    CuboidFluidSource(AABB bbox);
    CuboidFluidSource(glm::vec3 position, double w, double h, double d,
                      glm::vec3 velocity);
    CuboidFluidSource(AABB bbox, glm::vec3 velocity);
    ~CuboidFluidSource();

    virtual std::vector<GridIndex> getNewFluidCells(Array3d<int> &materialGrid,
                                                    double dx);
    virtual std::vector<GridIndex> getFluidCells(Array3d<int> &materialGrid,
                                                 double dx);

    void setWidth(double w);
    void setHeight(double h);
    void setDepth(double d);
    double getWidth();
    double getHeight();
    double getDepth();
    void setBoundingBox(AABB bbox);
    AABB getBoundingBox();
    void setCenter(glm::vec3 pos);
    glm::vec3 getCenter();
    void expand(double value);
    
    

private:
    AABB _bbox;

    void _getOverlappingGridIndices(std::vector<GridIndex> &storage,
                                    int i, int j, int k, double dx);
};

