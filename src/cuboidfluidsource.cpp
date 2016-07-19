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
#include "cuboidfluidsource.h"

CuboidFluidSource::CuboidFluidSource() {
}

CuboidFluidSource::CuboidFluidSource(vmath::vec3 position, 
                                     double w, double h, double d) : 
                                     FluidSource(), 
                                     _bbox(position, w, h, d) {
}

CuboidFluidSource::CuboidFluidSource(AABB bbox) : 
                                     FluidSource(), 
                                     _bbox(bbox) {
}

CuboidFluidSource::CuboidFluidSource(vmath::vec3 position, 
                                     double w, double h, double d,
                                     vmath::vec3 velocity) : 
                                     FluidSource(velocity), 
                                     _bbox(position, w, h, d) {
}

CuboidFluidSource::CuboidFluidSource(AABB bbox, vmath::vec3 velocity) : 
                                     FluidSource(velocity), 
                                     _bbox(bbox) {
}

CuboidFluidSource::~CuboidFluidSource() {
}

vmath::vec3 CuboidFluidSource::getPosition() {
    return _bbox.position;
}

void CuboidFluidSource::setPosition(vmath::vec3 p) {
    _bbox.position = p;
}

AABB CuboidFluidSource::getAABB() {
    return _bbox;
}

void CuboidFluidSource::setAABB(AABB bbox) {
    _bbox = bbox;
}

bool CuboidFluidSource::containsPoint(vmath::vec3 p) {
    return _bbox.isPointInside(p);
}

void CuboidFluidSource::setWidth(double w) {
    _bbox.width = w;
}

void CuboidFluidSource::setHeight(double h) {
    _bbox.height = h;
}

void CuboidFluidSource::setDepth(double d) {
    _bbox.depth = d;
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

vmath::vec3 CuboidFluidSource::getCenter() {
    return vmath::vec3(_bbox.position.x + 0.5*_bbox.width, 
                       _bbox.position.y + 0.5*_bbox.height, 
                       _bbox.position.z + 0.5*_bbox.depth);
}

void CuboidFluidSource::setCenter(vmath::vec3 pos) {
    vmath::vec3 c = getCenter();
    vmath::vec3 trans = pos - c;
    _bbox.position += trans;
}

void CuboidFluidSource::expand(double value) {
    _bbox.expand(value);
}

void CuboidFluidSource::_getOverlappingCells(GridIndexVector &indices, double dx) {
    int isize = indices.width;
    int jsize = indices.height;
    int ksize = indices.depth;
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(_bbox, dx, isize, jsize, ksize, &gmin, &gmax);

    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                indices.push_back(i, j, k);
            }
        }
    }
}
