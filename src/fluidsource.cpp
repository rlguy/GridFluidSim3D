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
#include "fluidsource.h"

int FluidSource::_IDCounter = 0;

FluidSource::FluidSource() {
    _initializeID();
}

FluidSource::FluidSource(vmath::vec3 vel) : _velocity(vel),
                                            _direction(vmath::normalize(_velocity)) {
    if (!(vmath::length(_velocity) > 0.0)) {
        _direction = vmath::vec3(1.0, 0.0, 0.0);
    }
    _initializeID();
}


FluidSource::~FluidSource() {
}

vmath::vec3 FluidSource::getVelocity() {
    return _velocity;
}

void FluidSource::setVelocity(vmath::vec3 v) {
    _velocity = v;
    if (vmath::length(v) > 0.0) {
        _direction = vmath::normalize(v);
    } else {
        _direction = vmath::vec3(1.0, 0.0, 0.0);
    }
}

vmath::vec3 FluidSource::getDirection() {
    return _direction;
}

void FluidSource::setDirection(vmath::vec3 dir) {
    float length = vmath::length(dir);
    if (!(length > 0.0f)) {
        return;
    }

    _direction = vmath::normalize(dir);
    _velocity = vmath::length(_velocity) * _direction;
}

void FluidSource::setAsInflow() {
    _sourceType = FluidSourceType::inflow;
}

void FluidSource::setAsOutflow() {
    _sourceType = FluidSourceType::outflow;
}

FluidSourceType FluidSource::getSourceType() {
    return _sourceType;
}

bool FluidSource::isInflow() {
    return _sourceType == FluidSourceType::inflow;
}

bool FluidSource::isOutflow() {
    return _sourceType == FluidSourceType::outflow;
}

void FluidSource::activate() {
    _isActive = true;
}

void FluidSource::deactivate() {
    _isActive = false;
}

bool FluidSource::isActive() {
    return _isActive;
}

GridIndexVector FluidSource::getAirCells(FluidMaterialGrid &materialGrid,
                                                              double dx) {
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    GridIndexVector cells(w, h, d);

    if (!isActive()) {
        return cells;
    }

    GridIndexVector overlappingIndices(w, h, d);
    _getOverlappingCells(overlappingIndices, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid.isCellAir(g)) {
            cells.push_back(g);
        }
    }

    return cells;
}

GridIndexVector FluidSource::getFluidCells(FluidMaterialGrid &materialGrid,
                                                        double dx) {
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    GridIndexVector cells(w, h, d);

    if (!isActive()) {
        return cells;
    }

    GridIndexVector overlappingIndices(w, h, d);
    _getOverlappingCells(overlappingIndices, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid.isCellFluid(g)) {
            cells.push_back(g);
        }
    }

    return cells;
}

GridIndexVector FluidSource::getFluidOrAirCells(FluidMaterialGrid &materialGrid,
                                                   double dx) {
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    GridIndexVector cells(w, h, d);

    if (!isActive()) {
        return cells;
    }

    GridIndexVector overlappingIndices(w, h, d);
    _getOverlappingCells(overlappingIndices, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (!materialGrid.isCellSolid(g)) {
            cells.push_back(g);
        }
    }

    return cells;
}

int FluidSource::getID() {
    return _ID;
}

void FluidSource::_initializeID() {
    _ID = _IDCounter++;
}