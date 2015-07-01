#pragma once

#include <vector>

#include "glm/glm.hpp"
#include "array3d.h"
#include "aabb.h"

class FluidSource
{
public:
    FluidSource();
    FluidSource(glm::vec3 pos);
    FluidSource(glm::vec3 pos, glm::vec3 velocity);
    ~FluidSource();

    void setPosition(glm::vec3 pos);
    glm::vec3 getPosition();
    void translate(glm::vec3 trans);
    void setVelocity(glm::vec3 v);
    void setDirection(glm::vec3 dir);
    glm::vec3 getVelocity();
    void setAsInFlow();
    void setAsOutFlow();
    int getSourceType();
    void activate();
    void deactivate();

    virtual std::vector<GridIndex> getNewFluidCells(Array3d<int> &materialGrid,
                                                    double dx);

protected:
    int T_INFLOW = 0;
    int T_OUTFLOW = 1;
    int M_AIR = 0;
    int M_FLUID = 1;
    int M_SOLID = 2;

    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 direction;

    bool isActive = true;
    int sourceType = T_INFLOW;

    inline GridIndex positionToGridIndex(glm::vec3 p, double dx) {
        double invdx = 1.0 / dx;
        return GridIndex(floor(p.x*invdx), floor(p.y*invdx), floor(p.z*invdx));
    }

    inline glm::vec3 GridIndexToCellCenter(int i, int j, int k, double dx) {
        return glm::vec3(i*dx + 0.5*dx, j*dx + 0.5*dx, k*dx + 0.5*dx);
    }

    void getGridIndexBounds(AABB bbox, int i, int j, int k, double dx, 
                            GridIndex *gmin, GridIndex *gmax);

};

