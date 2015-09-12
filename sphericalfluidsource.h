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
    void setCenter(glm::vec3 p);
    void expand(double val);

    virtual std::vector<GridIndex> getNewFluidCells(Array3d<int> &materialGrid,
                                                    double dx);
    virtual std::vector<GridIndex> getFluidCells(Array3d<int> &materialGrid,
                                                 double dx);

private:

    void _getOverlappingGridIndices(std::vector<GridIndex> &storage,
                                    int i, int j, int k, double dx);

    double _radius = 0.0;

};

