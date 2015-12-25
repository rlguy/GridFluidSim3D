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
#include "cuboidfluidsource.h"


CuboidFluidSource::CuboidFluidSource() {
}

CuboidFluidSource::CuboidFluidSource(vmath::vec3 position, 
                                     double w, double h, double d) : 
                                     FluidSource(position), 
                                     _bbox(position, w, h, d) {
}

CuboidFluidSource::CuboidFluidSource(AABB bbox) : 
                                     FluidSource(bbox.position), 
                                     _bbox(bbox) {
}

CuboidFluidSource::CuboidFluidSource(vmath::vec3 position, 
                                     double w, double h, double d,
                                     vmath::vec3 velocity) : 
                                     FluidSource(position, velocity), 
                                     _bbox(position, w, h, d) {
}

CuboidFluidSource::CuboidFluidSource(AABB bbox, vmath::vec3 velocity) : 
                                     FluidSource(bbox.position, velocity), 
                                     _bbox(bbox) {
}

CuboidFluidSource::~CuboidFluidSource() {
}

void CuboidFluidSource::setWidth(double w) {
    _bbox = AABB(position, w, _bbox.height, _bbox.depth);
}

void CuboidFluidSource::setHeight(double h) {
    _bbox = AABB(position, _bbox.width, h, _bbox.depth);
}

void CuboidFluidSource::setDepth(double d) {
    _bbox = AABB(position, _bbox.width, _bbox.height, d);
}

double CuboidFluidSource::getWidth() {
    return _bbox.width;
}

double CuboidFluidSource::getHeight() {
    return _bbox.height;
}

double CuboidFluidSource::getDepth() {
    return _bbox.depth;
}

void CuboidFluidSource::setBoundingBox(AABB bbox) {
    _bbox = bbox;
    position = bbox.position;
}

AABB CuboidFluidSource::getBoundingBox() {
    _bbox.position = position;
    return _bbox;
}

void CuboidFluidSource::setCenter(vmath::vec3 pos) {
    vmath::vec3 c = getCenter();
    translate(pos - c);
    _bbox.position = pos;
}

vmath::vec3 CuboidFluidSource::getCenter() {
    return vmath::vec3(position.x + 0.5*_bbox.width, 
                     position.y + 0.5*_bbox.width, 
                     position.z + 0.5*_bbox.width);
}

void CuboidFluidSource::expand(double value) {
    vmath::vec3 p = position - vmath::vec3(value, value, value);
    _bbox = AABB(p, _bbox.width + 2*value, 
                    _bbox.height + 2*value, 
                    _bbox.depth + 2*value);
}

GridIndexVector CuboidFluidSource::getNewFluidCells(FluidMaterialGrid &materialGrid,
                                                              double dx) {
    if (!isActive) {
        return GridIndexVector();
    }

    if (sourceType == T_OUTFLOW) {
        return GridIndexVector();
    }

    _bbox.position = position;
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    GridIndexVector overlappingIndices(w, h, d);
    GridIndexVector newFluidCells(w, h, d);

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid.isCellAir(g)) {
            newFluidCells.push_back(g);
        }
    }

    return newFluidCells;
}

GridIndexVector CuboidFluidSource::getFluidCells(FluidMaterialGrid &materialGrid,
                                                        double dx) {
    if (!isActive) {
        return GridIndexVector();
    }

     _bbox.position = position;
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    GridIndexVector overlappingIndices(w, h, d);
    GridIndexVector fluidCells(w, h, d);

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid.isCellFluid(g)) {
            fluidCells.push_back(g);
        }
    }

    return fluidCells;
}

GridIndexVector CuboidFluidSource::getCells(FluidMaterialGrid &materialGrid,
                                                   double dx) {
    if (!isActive) {
        return GridIndexVector();
    }

    if (sourceType == T_OUTFLOW) {
        return GridIndexVector();
    }

     _bbox.position = position;
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    GridIndexVector overlappingIndices(w, h, d);
    GridIndexVector cells(w, h, d);

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (!materialGrid.isCellSolid(g)) {
            cells.push_back(g);
        }
    }

    return cells;
}

void CuboidFluidSource::_getOverlappingGridIndices(GridIndexVector &indices,
                                                   int isize, int jsize, int ksize, 
                                                   double dx) {
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(_bbox, dx, isize, jsize, ksize, &gmin, &gmax);

    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                indices.push_back(GridIndex(i, j, k));
            }
        }
    }
}

AABB CuboidFluidSource::getAABB() {
    return _bbox;
}