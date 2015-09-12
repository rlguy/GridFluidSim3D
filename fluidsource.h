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

#include <vector>
#include <stdio.h>
#include <iostream>

#include "glm/glm.hpp"
#include "array3d.h"
#include "grid3d.h"
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
    int getID();
    void setID(int identifier);

    virtual std::vector<GridIndex> getNewFluidCells(Array3d<int> &materialGrid,
                                                    double dx);
    virtual std::vector<GridIndex> getFluidCells(Array3d<int> &materialGrid,
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
    int id = 0;

};

