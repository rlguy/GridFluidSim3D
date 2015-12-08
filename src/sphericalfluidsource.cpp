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
#include "sphericalfluidsource.h"


SphericalFluidSource::SphericalFluidSource() {
}

SphericalFluidSource::SphericalFluidSource(glm::vec3 pos, double r) : 
                                           FluidSource(pos),
                                           _radius(r > 0.0 ? r : 0.0) {
}

SphericalFluidSource::SphericalFluidSource(glm::vec3 pos, double r, glm::vec3 velocity) : 
                                           FluidSource(pos, velocity),
                                           _radius(r > 0.0 ? r : 0.0) {
}

SphericalFluidSource::~SphericalFluidSource() {
}

void SphericalFluidSource::setRadius(double r) {
    if (r < 0.0) {
        r = 0.0;
    }
    _radius = r;
}

double SphericalFluidSource::getRadius() {
    return _radius;
}

void SphericalFluidSource::setCenter(glm::vec3 p) {
    position = p;
}

void SphericalFluidSource::expand(double val) {
    _radius += val;

    if (_radius < 0.0) {
        _radius = 0.0;
    }
}

std::vector<GridIndex> SphericalFluidSource::getNewFluidCells(Array3d<int> &materialGrid,
                                                              double dx) {
    if (!isActive) {
        return std::vector<GridIndex>();
    }

    if (sourceType == T_OUTFLOW) {
        return std::vector<GridIndex>();
    }

    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    std::vector<GridIndex> overlappingIndices;
    std::vector<GridIndex> newFluidCells;

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid(g) == M_AIR) {
            newFluidCells.push_back(g);
        }
    }

    return newFluidCells;
}

std::vector<GridIndex> SphericalFluidSource::getFluidCells(Array3d<int> &materialGrid,
                                                           double dx) {
    if (!isActive) {
        return std::vector<GridIndex>();
    }

    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    std::vector<GridIndex> overlappingIndices;
    std::vector<GridIndex> fluidCells;

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid(g) == M_FLUID) {
            fluidCells.push_back(g);
        }
    }

    return fluidCells;
}

std::vector<GridIndex> SphericalFluidSource::getCells(Array3d<int> &materialGrid,
                                                   double dx) {
    if (!isActive) {
        return std::vector<GridIndex>();
    }

    if (sourceType == T_OUTFLOW) {
        return std::vector<GridIndex>();
    }

    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    std::vector<GridIndex> overlappingIndices;
    std::vector<GridIndex> cells;

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (unsigned int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid(g) != M_SOLID) {
            cells.push_back(g);
        }
    }

    return cells;
}

void SphericalFluidSource::_getOverlappingGridIndices(std::vector<GridIndex> &indices,
                                                      int isize, int jsize, int ksize, 
                                                      double dx) {

    double r = _radius;
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(position, r, dx, isize, jsize, ksize, &gmin, &gmax);

    double rsq = r*r;
    double distsq;
    glm::vec3 p;
    glm::vec3 v;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                p = Grid3d::GridIndexToCellCenter(i, j, k, dx);
                v = p - position;
                distsq = glm::dot(v, v);
                if (distsq < rsq) {
                    indices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

}

AABB SphericalFluidSource::getAABB() {
    double d = 2.0*_radius;
    glm::vec3 p = position - glm::vec3(_radius, _radius, _radius);
    return AABB(p, d, d, d);
}