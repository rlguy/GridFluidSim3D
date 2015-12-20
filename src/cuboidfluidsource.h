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
#ifndef CUBOIDFLUIDSOURCE_H
#define CUBOIDFLUIDSOURCE_H

#include "fluidsource.h"

#include <vector>

#include "vmath.h"
#include "aabb.h"
#include "fluidmaterialgrid.h"

class CuboidFluidSource : public FluidSource
{
public:
    CuboidFluidSource();
    CuboidFluidSource(vmath::vec3 position, double w, double h, double d);
    CuboidFluidSource(AABB bbox);
    CuboidFluidSource(vmath::vec3 position, double w, double h, double d,
                      vmath::vec3 velocity);
    CuboidFluidSource(AABB bbox, vmath::vec3 velocity);
    ~CuboidFluidSource();

    virtual std::vector<GridIndex> getNewFluidCells(FluidMaterialGrid &materialGrid,
                                                    double dx);
    virtual std::vector<GridIndex> getFluidCells(FluidMaterialGrid &materialGrid,
                                                 double dx);
    virtual std::vector<GridIndex> getCells(FluidMaterialGrid &materialGrid,
                                            double dx);
    virtual AABB getAABB();

    void setWidth(double w);
    void setHeight(double h);
    void setDepth(double d);
    double getWidth();
    double getHeight();
    double getDepth();
    void setBoundingBox(AABB bbox);
    AABB getBoundingBox();
    void setCenter(vmath::vec3 pos);
    vmath::vec3 getCenter();
    void expand(double value);
    
private:
    AABB _bbox;

    void _getOverlappingGridIndices(std::vector<GridIndex> &storage,
                                    int i, int j, int k, double dx);
};

#endif
