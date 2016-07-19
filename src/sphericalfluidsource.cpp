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
#include "sphericalfluidsource.h"

SphericalFluidSource::SphericalFluidSource() {
}

SphericalFluidSource::SphericalFluidSource(vmath::vec3 pos, double r) : 
                                           FluidSource(),
                                           _position(pos),
                                           _radius(r > 0.0 ? r : 0.0) {
}

SphericalFluidSource::SphericalFluidSource(vmath::vec3 pos, double r, vmath::vec3 velocity) : 
                                           FluidSource(velocity),
                                           _position(pos),
                                           _radius(r > 0.0 ? r : 0.0) {
}

SphericalFluidSource::~SphericalFluidSource() {
}

vmath::vec3 SphericalFluidSource::getPosition() {
    return _position;
}

void SphericalFluidSource::setPosition(vmath::vec3 pos) {
    _position = pos;
}

AABB SphericalFluidSource::getAABB() {
    double d = 2.0*_radius;
    vmath::vec3 p = _position - vmath::vec3(_radius, _radius, _radius);
    return AABB(p, d, d, d);
}

bool SphericalFluidSource::containsPoint(vmath::vec3 p) {
    vmath::vec3 v = p - getPosition();
    double lensq = vmath::lengthsq(v);
    return lensq < _radius*_radius;
}

double SphericalFluidSource::getRadius() {
    return _radius;
}

void SphericalFluidSource::setRadius(double r) {
    if (r < 0.0) {
        r = 0.0;
    }
    _radius = r;
}

void SphericalFluidSource::setCenter(vmath::vec3 p) {
    _position = p;
}

void SphericalFluidSource::expand(double val) {
    _radius += 0.5 * val;

    if (_radius < 0.0) {
        _radius = 0.0;
    }
}

void SphericalFluidSource::_getOverlappingCells(GridIndexVector &indices, double dx) {
    int isize = indices.width;
    int jsize = indices.height;
    int ksize = indices.depth;
    double r = _radius;
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(_position, r, dx, isize, jsize, ksize, &gmin, &gmax);

    double rsq = r*r;
    double distsq;
    vmath::vec3 p;
    vmath::vec3 v;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                p = Grid3d::GridIndexToCellCenter(i, j, k, dx);
                v = p - _position;
                distsq = vmath::dot(v, v);
                if (distsq < rsq) {
                    indices.push_back(i, j, k);
                }
            }
        }
    }

}
