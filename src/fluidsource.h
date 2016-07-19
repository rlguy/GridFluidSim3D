/*
Copyright (c) 2016 Ryan L. Guy

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
#ifndef FLUIDSOURCE_H
#define FLUIDSOURCE_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include "vmath.h"
#include "array3d.h"
#include "grid3d.h"
#include "aabb.h"
#include "fluidmaterialgrid.h"
#include "gridindexvector.h"

enum class FluidSourceType : char { 
    inflow   = 0x00, 
    outflow = 0x01
};

class FluidSource
{
public:
    FluidSource();
    FluidSource(vmath::vec3 velocity);
    virtual ~FluidSource();

    virtual vmath::vec3 getPosition() = 0;
    virtual void setPosition(vmath::vec3 pos) = 0;
    virtual AABB getAABB() = 0;
    virtual bool containsPoint(vmath::vec3 p) = 0;

    vmath::vec3 getVelocity();
    void setVelocity(vmath::vec3 v);
    vmath::vec3 getDirection();
    void setDirection(vmath::vec3 dir);
    void setAsInflow();
    void setAsOutflow();
    FluidSourceType getSourceType();
    bool isInflow();
    bool isOutflow();
    void activate();
    void deactivate();
    bool isActive();
    int getID();

    GridIndexVector getAirCells(FluidMaterialGrid &materialGrid, double dx);
    GridIndexVector getFluidCells(FluidMaterialGrid &materialGrid, double dx);
    GridIndexVector getFluidOrAirCells(FluidMaterialGrid &materialGrid, double dx);

protected:
    virtual void _getOverlappingCells(GridIndexVector &storage, double dx) = 0;
    
private:
    void _initializeID();

    vmath::vec3 _velocity;
    vmath::vec3 _direction;
    bool _isActive = true;
    FluidSourceType _sourceType = FluidSourceType::inflow;
    int _ID;

    static int _IDCounter;
};

#endif
