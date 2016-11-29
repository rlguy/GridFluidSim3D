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
#include "macvelocityfield.h"

MACVelocityField::MACVelocityField() {
    _initializeVelocityGrids();
}

MACVelocityField::MACVelocityField(int isize, int jsize, int ksize, double dx) :
                                   _isize(isize), _jsize(jsize), _ksize(ksize),
                                   _dx(dx) {

    _initializeVelocityGrids();
}


MACVelocityField::~MACVelocityField() {
}

void MACVelocityField::_initializeVelocityGrids() {
    _u = Array3d<float>(_isize + 1, _jsize, _ksize, 0.0f);
    _v = Array3d<float>(_isize, _jsize + 1, _ksize, 0.0f);
    _w = Array3d<float>(_isize, _jsize, _ksize + 1, 0.0f);

    _u.setOutOfRangeValue(0.0f);
    _v.setOutOfRangeValue(0.0f);
    _w.setOutOfRangeValue(0.0f);
}

void MACVelocityField::getGridDimensions(int *i, int *j, int *k) {
    *i = _isize;
    *j = _jsize;
    *k = _ksize;
}

double MACVelocityField::getGridCellSize() {
    return _dx;
}

void MACVelocityField::clearU() {
    _u.fill(0.0);
}

void MACVelocityField::clearV() {
    _v.fill(0.0);
}

void MACVelocityField::clearW() {
    _w.fill(0.0);
}

void MACVelocityField::clear() {
    clearU();
    clearV();
    clearW();
}

Array3d<float>* MACVelocityField::getArray3dU() {
    return &_u;
}

Array3d<float>* MACVelocityField::getArray3dV() {
    return &_v;
}

Array3d<float>* MACVelocityField::getArray3dW() {
    return &_w;
}

float* MACVelocityField::getRawArrayU() {
    return _u.getRawArray();
}

float* MACVelocityField::getRawArrayV() {
    return _v.getRawArray();
}

float* MACVelocityField::getRawArrayW() {
    return _w.getRawArray();
}

float MACVelocityField::U(int i, int j, int k) {
    if (!isIndexInRangeU(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _u(i, j, k);
}

float MACVelocityField::V(int i, int j, int k) {
    if (!isIndexInRangeV(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _v(i, j, k);
}

float MACVelocityField::W(int i, int j, int k) {
    if (!isIndexInRangeW(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _w(i, j, k);
}

float MACVelocityField::U(GridIndex g) {
    if (!isIndexInRangeU(g)) {
        return _default_out_of_range_value;
    }

    return _u(g);
}

float MACVelocityField::V(GridIndex g) {
    if (!isIndexInRangeV(g)) {
        return _default_out_of_range_value;
    }

    return _v(g);
}

float MACVelocityField::W(GridIndex g) {
    if (!isIndexInRangeW(g)) {
        return _default_out_of_range_value;
    }

    return _w(g);
}

void MACVelocityField::setU(int i, int j, int k, double val) {
    if (!isIndexInRangeU(i, j, k)) {
        return;
    }

    _u.set(i, j, k, (float)val);
}

void MACVelocityField::setV(int i, int j, int k, double val) {
    if (!isIndexInRangeV(i, j, k)) {
        return;
    }

    _v.set(i, j, k, (float)val);
}

void MACVelocityField::setW(int i, int j, int k, double val) {
    if (!isIndexInRangeW(i, j, k)) {
        return;
    }

    _w.set(i, j, k, (float)val);
}

void MACVelocityField::setU(GridIndex g, double val) {
    setU(g.i, g.j, g.k, val);
}

void MACVelocityField::setV(GridIndex g, double val) {
    setV(g.i, g.j, g.k, val);
}

void MACVelocityField::setW(GridIndex g, double val) {
    setW(g.i, g.j, g.k, val);
}

void MACVelocityField::setU(Array3d<float> &ugrid) {
    FLUIDSIM_ASSERT(ugrid.width == _u.width && 
           ugrid.height == _u.height && 
           ugrid.depth == _u.depth);
    _u = ugrid;
}

void MACVelocityField::setV(Array3d<float> &vgrid) {
    FLUIDSIM_ASSERT(vgrid.width == _v.width && 
           vgrid.height == _v.height && 
           vgrid.depth == _v.depth);
    _v = vgrid;
}

void MACVelocityField::setW(Array3d<float> &wgrid) {
    FLUIDSIM_ASSERT(wgrid.width == _w.width && 
           wgrid.height == _w.height && 
           wgrid.depth == _w.depth);
    _w = wgrid;
}

void MACVelocityField::addU(int i, int j, int k, double val) {
    if (!isIndexInRangeU(i, j, k)) {
        return;
    }

    _u.add(i, j, k, (float)val);
}

void MACVelocityField::addV(int i, int j, int k, double val) {
    if (!isIndexInRangeV(i, j, k)) {
        return;
    }

    _v.add(i, j, k, (float)val);
}

void MACVelocityField::addW(int i, int j, int k, double val) {
    if (!isIndexInRangeW(i, j, k)) {
        return;
    }

    _w.add(i, j, k, (float)val);
}

vmath::vec3 MACVelocityField::velocityIndexToPositionU(int i, int j, int k) {
    FLUIDSIM_ASSERT(isIndexInRangeU(i, j, k));

    double gx = (double)(i-1)*_dx;
    double gy = (double)j*_dx;
    double gz = (double)k*_dx;

    return vmath::vec3(gx + _dx, gy + 0.5*_dx, gz + 0.5*_dx);
}

vmath::vec3 MACVelocityField::velocityIndexToPositionV(int i, int j, int k) {
    FLUIDSIM_ASSERT(isIndexInRangeV(i, j, k));

    double gx = (double)i*_dx;
    double gy = (double)(j-1)*_dx;
    double gz = (double)k*_dx;

    return vmath::vec3(gx + 0.5*_dx, gy + _dx, gz + 0.5*_dx);
}

vmath::vec3 MACVelocityField::velocityIndexToPositionW(int i, int j, int k) {
    FLUIDSIM_ASSERT(isIndexInRangeW(i, j, k));

    double gx = (float)i*_dx;
    double gy = (float)j*_dx;
    double gz = (float)(k-1)*_dx;

    return vmath::vec3(gx + 0.5f*_dx, gy + 0.5f*_dx, gz + _dx);
}

vmath::vec3 MACVelocityField::evaluateVelocityAtCellCenter(int i, int j, int k) {
    FLUIDSIM_ASSERT(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i, j + 1, k) + V(i, j, k));
    double zavg = 0.5 * (W(i, j, k + 1) + W(i, j, k));

    return vmath::vec3(xavg, yavg, zavg);
}

float MACVelocityField::evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k) {
    FLUIDSIM_ASSERT(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i, j + 1, k) + V(i, j, k));
    double zavg = 0.5 * (W(i, j, k + 1) + W(i, j, k));

    return (float)(xavg*xavg + yavg*yavg + zavg*zavg);
}

float MACVelocityField::evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k) {
    FLUIDSIM_ASSERT(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double mag = evaluateVelocityMagnitudeSquaredAtCellCenter(i, j, k);
    if (mag > 0.0) {
        return (float)sqrt(mag);
    }
    else {
        return 0.0;
    }
}

float MACVelocityField::evaluateMaximumVelocityMagnitude() {
    double maxsq = 0.0;
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                
                double m = evaluateVelocityMagnitudeSquaredAtCellCenter(i, j, k);
                maxsq = fmax(maxsq, m);
            }
        }
    }

    double max = maxsq;
    if (maxsq > 0.0) {
        max = sqrt(maxsq);
    }

    return (float)max;
}

vmath::vec3 MACVelocityField::evaluateVelocityAtFaceCenterU(int i, int j, int k) {
    FLUIDSIM_ASSERT(isIndexInRangeU(i, j, k));

    // Shift reference coordinate to the left. The formula used is for calculating
    // u(i+1/2, j, k). If we keep original (i,j,k) coordinate, then using the formula
    // would calculate u(i+3/2, j, k) instead. The same will be done for the V and W
    // faces, shifting back in the respective direction.
    i--;

    double vx = U(i+1, j, k);
    double vy = 0.25 * (V(i, j, k) + V(i, j+1, k) + V(i+1, j, k) + V(i+1, j+1, k));
    double vz = 0.25 * (W(i, j, k) + W(i, j, k+1) + W(i+1, j, k) + W(i+1, j, k+1));

    return vmath::vec3(vx, vy, vz);
}

vmath::vec3 MACVelocityField::evaluateVelocityAtFaceCenterV(int i, int j, int k) {
    FLUIDSIM_ASSERT(isIndexInRangeV(i, j, k));

    j--;

    double vx = 0.25 * (U(i, j, k) + U(i+1, j, k) + U(i, j+1, k) + U(i+1, j+1, k));
    double vy = V(i, j + 1, k);
    double vz = 0.25 * (W(i, j, k) + W(i, j, k+1) + W(i, j+1, k) + W(i, j+1, k+1));

    return vmath::vec3(vx, vy, vz);
}

vmath::vec3 MACVelocityField::evaluateVelocityAtFaceCenterW(int i, int j, int k) {
    FLUIDSIM_ASSERT(isIndexInRangeW(i, j, k));

    k--;

    double vx = 0.25 * (U(i, j, k) + U(i+1, j, k) + U(i, j, k+1) + U(i+1, j, k+1));
    double vy = 0.25 * (V(i, j, k) + V(i, j+1, k) + V(i, j, k+1) + V(i, j+1, k+1));
    double vz = W(i, j, k + 1);

    return vmath::vec3(vx, vy, vz);
}

double MACVelocityField::_interpolateU(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return 0.0;
    }

    y -= 0.5*_dx;
    z -= 0.5*_dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx)*inv_dx;
    double iy = (y - gy)*inv_dx;
    double iz = (z - gz)*inv_dx;

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;

    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                points[pk][pj][pi] = U(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return Interpolation::tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateV(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return 0.0;
    }

    x -= 0.5*_dx;
    z -= 0.5*_dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx)*inv_dx;
    double iy = (y - gy)*inv_dx;
    double iz = (z - gz)*inv_dx;

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;

    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                points[pk][pj][pi] = V(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return Interpolation::tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateW(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return 0.0;
    }

    x -= 0.5*_dx;
    y -= 0.5*_dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx)*inv_dx;
    double iy = (y - gy)*inv_dx;
    double iz = (z - gz)*inv_dx;

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;

    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                points[pk][pj][pi] = W(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return Interpolation::tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateLinearU(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return 0.0;
    }

    y -= 0.5*_dx;
    z -= 0.5*_dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx)*inv_dx;
    double iy = (y - gy)*inv_dx;
    double iz = (z - gz)*inv_dx;

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_u.isIndexInRange(i,   j,   k))   { points[0] = _u(i,   j,   k); }
    if (_u.isIndexInRange(i+1, j,   k))   { points[1] = _u(i+1, j,   k); }
    if (_u.isIndexInRange(i,   j+1, k))   { points[2] = _u(i,   j+1, k); }
    if (_u.isIndexInRange(i,   j,   k+1)) { points[3] = _u(i,   j,   k+1); }
    if (_u.isIndexInRange(i+1, j,   k+1)) { points[4] = _u(i+1, j,   k+1); }
    if (_u.isIndexInRange(i,   j+1, k+1)) { points[5] = _u(i,   j+1, k+1); }
    if (_u.isIndexInRange(i+1, j+1, k))   { points[6] = _u(i+1, j+1, k); }
    if (_u.isIndexInRange(i+1, j+1, k+1)) { points[7] = _u(i+1, j+1, k+1); }

    return Interpolation::trilinearInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateLinearV(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return 0.0;
    }

    x -= 0.5*_dx;
    z -= 0.5*_dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx)*inv_dx;
    double iy = (y - gy)*inv_dx;
    double iz = (z - gz)*inv_dx;

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_v.isIndexInRange(i,   j,   k))   { points[0] = _v(i,   j,   k); }
    if (_v.isIndexInRange(i+1, j,   k))   { points[1] = _v(i+1, j,   k); }
    if (_v.isIndexInRange(i,   j+1, k))   { points[2] = _v(i,   j+1, k); }
    if (_v.isIndexInRange(i,   j,   k+1)) { points[3] = _v(i,   j,   k+1); }
    if (_v.isIndexInRange(i+1, j,   k+1)) { points[4] = _v(i+1, j,   k+1); }
    if (_v.isIndexInRange(i,   j+1, k+1)) { points[5] = _v(i,   j+1, k+1); }
    if (_v.isIndexInRange(i+1, j+1, k))   { points[6] = _v(i+1, j+1, k); }
    if (_v.isIndexInRange(i+1, j+1, k+1)) { points[7] = _v(i+1, j+1, k+1); }

    return Interpolation::trilinearInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateLinearW(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return 0.0;
    }

    x -= 0.5*_dx;
    y -= 0.5*_dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx)*inv_dx;
    double iy = (y - gy)*inv_dx;
    double iz = (z - gz)*inv_dx;

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_w.isIndexInRange(i,   j,   k))   { points[0] = _w(i,   j,   k); }
    if (_w.isIndexInRange(i+1, j,   k))   { points[1] = _w(i+1, j,   k); }
    if (_w.isIndexInRange(i,   j+1, k))   { points[2] = _w(i,   j+1, k); }
    if (_w.isIndexInRange(i,   j,   k+1)) { points[3] = _w(i,   j,   k+1); }
    if (_w.isIndexInRange(i+1, j,   k+1)) { points[4] = _w(i+1, j,   k+1); }
    if (_w.isIndexInRange(i,   j+1, k+1)) { points[5] = _w(i,   j+1, k+1); }
    if (_w.isIndexInRange(i+1, j+1, k))   { points[6] = _w(i+1, j+1, k); }
    if (_w.isIndexInRange(i+1, j+1, k+1)) { points[7] = _w(i+1, j+1, k+1); }

    return Interpolation::trilinearInterpolate(points, ix, iy, iz);
}

vmath::vec3 MACVelocityField::evaluateVelocityAtPosition(vmath::vec3 pos) {
    return evaluateVelocityAtPosition(pos.x, pos.y, pos.z);
}

vmath::vec3 MACVelocityField::evaluateVelocityAtPosition(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return vmath::vec3();
    }

    double xvel = _interpolateU(x, y, z);
    double yvel = _interpolateV(x, y, z);
    double zvel = _interpolateW(x, y, z);

    return vmath::vec3(xvel, yvel, zvel);
}

vmath::vec3 MACVelocityField::evaluateVelocityAtPositionLinear(vmath::vec3 pos) {
    return evaluateVelocityAtPositionLinear(pos.x, pos.y, pos.z);
}

vmath::vec3 MACVelocityField::evaluateVelocityAtPositionLinear(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return vmath::vec3();
    }

    double xvel = _interpolateLinearU(x, y, z);
    double yvel = _interpolateLinearV(x, y, z);
    double zvel = _interpolateLinearW(x, y, z);

    return vmath::vec3(xvel, yvel, zvel);
}

void MACVelocityField::_updateExtrapolationLayer(int layerIndex, 
                                                 FluidMaterialGrid &matGrid,
                                                 Array3d<int> &layerGrid) {
    GridIndex neighbours[6];
    GridIndex n;

    for (int k = 0; k < layerGrid.depth; k++) {
        for (int j = 0; j < layerGrid.height; j++) {
            for (int i = 0; i < layerGrid.width; i++) {
                if (layerGrid(i, j, k) == layerIndex - 1 && !matGrid.isCellSolid(i, j, k)) {
                    Grid3d::getNeighbourGridIndices6(i, j, k, neighbours);
                    for (int idx = 0; idx < 6; idx++) {
                        n = neighbours[idx];

                        if (Grid3d::isGridIndexInRange(n, _isize, _jsize, _ksize) && 
                                layerGrid(n) == -1 && !matGrid.isCellSolid(n)) {
                            layerGrid.set(n, layerIndex);
                        }
                    }
                }
            }
        }
    }
}

void MACVelocityField::_updateExtrapolationLayers(FluidMaterialGrid &matGrid, 
                                                  Array3d<int> &layerGrid) {

    for (int k = 0; k < matGrid.depth; k++) {
        for (int j = 0; j < matGrid.height; j++) {
            for (int i = 0; i < matGrid.width; i++) {
                if (matGrid.isCellFluid(i, j, k)) {
                    layerGrid.set(i, j, k, 0);
                }
            }
        }
    }

    for (int layer = 1; layer <= _numExtrapolationLayers; layer++) {
        _updateExtrapolationLayer(layer, matGrid, layerGrid);
    }
}

double MACVelocityField::_getExtrapolatedVelocityForFaceU(int i, int j, int k, int layerIdx,
                                                          Array3d<int> &layerGrid) {
    GridIndex n[6];
    Grid3d::getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (isIndexInRangeU(c) && _isFaceBorderingLayerIndexU(c, layerIdx - 1, layerGrid)) {
                sum += U(c);
                weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

double MACVelocityField::_getExtrapolatedVelocityForFaceV(int i, int j, int k, int layerIdx,
                                                          Array3d<int> &layerGrid) {
    GridIndex n[6];
    Grid3d::getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (isIndexInRangeV(c) && _isFaceBorderingLayerIndexV(c, layerIdx - 1, layerGrid)) {
            sum += V(c);
            weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

double MACVelocityField::_getExtrapolatedVelocityForFaceW(int i, int j, int k, int layerIdx,
                                                          Array3d<int> &layerGrid) {
    GridIndex n[6];
    Grid3d::getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (isIndexInRangeW(c) && _isFaceBorderingLayerIndexW(c, layerIdx - 1, layerGrid)) {
            sum += W(c);
            weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

void MACVelocityField::_extrapolateVelocitiesForLayerIndexU(int idx, 
                                                            FluidMaterialGrid &matGrid,
                                                            Array3d<int> &layerGrid) {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                bool isExtrapolated = _isFaceBorderingLayerIndexU(i, j, k, idx, layerGrid) && 
                                     !_isFaceBorderingLayerIndexU(i, j, k, idx-1, layerGrid) &&
                                    (!matGrid.isFaceBorderingSolidU(i, j, k));
                if (isExtrapolated) {
                    double v = _getExtrapolatedVelocityForFaceU(i, j, k, idx, layerGrid);
                    setU(i, j, k, (float)v);
                }
            }
        }
    }
}

void MACVelocityField::_extrapolateVelocitiesForLayerIndexV(int idx, 
                                                            FluidMaterialGrid &matGrid,
                                                            Array3d<int> &layerGrid) {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                bool isExtrapolated = _isFaceBorderingLayerIndexV(i, j, k, idx, layerGrid) && 
                                     !_isFaceBorderingLayerIndexV(i, j, k, idx-1, layerGrid) &&
                                    (!matGrid.isFaceBorderingSolidV(i, j, k));
                if (isExtrapolated) {
                    double v = _getExtrapolatedVelocityForFaceV(i, j, k, idx, layerGrid);
                    setV(i, j, k, (float)v);
                }
            }
        }
    }
}

void MACVelocityField::_extrapolateVelocitiesForLayerIndexW(int idx, 
                                                            FluidMaterialGrid &matGrid,
                                                            Array3d<int> &layerGrid) {
    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                bool isExtrapolated = _isFaceBorderingLayerIndexW(i, j, k, idx, layerGrid) && 
                                     !_isFaceBorderingLayerIndexW(i, j, k, idx-1, layerGrid) &&
                                    (!matGrid.isFaceBorderingSolidW(i, j, k));
                if (isExtrapolated) {
                    double v = _getExtrapolatedVelocityForFaceW(i, j, k, idx, layerGrid);
                    setW(i, j, k, (float)v);
                }
            }
        }
    }
}

void MACVelocityField::_extrapolateVelocitiesForLayerIndex(int idx, 
                                                           FluidMaterialGrid &matGrid,
                                                           Array3d<int> &layerGrid) {
    _extrapolateVelocitiesForLayerIndexU(idx, matGrid, layerGrid);
    _extrapolateVelocitiesForLayerIndexV(idx, matGrid, layerGrid);
    _extrapolateVelocitiesForLayerIndexW(idx, matGrid, layerGrid);
}

void MACVelocityField::_resetExtrapolatedFluidVelocities(FluidMaterialGrid &matGrid) {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                if (!matGrid.isFaceBorderingFluidU(i, j, k)) {
                    setU(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                if (!matGrid.isFaceBorderingFluidV(i, j, k)) {
                    setV(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (!matGrid.isFaceBorderingFluidW(i, j, k)) {
                    setW(i, j, k, 0.0);
                }
            }
        }
    }
}

void MACVelocityField::extrapolateVelocityField(FluidMaterialGrid &materialGrid, 
                                                int numLayers) {
    _numExtrapolationLayers = numLayers;

    Array3d<int> layerGrid = Array3d<int>(_isize, _jsize, _ksize, -1);

    _resetExtrapolatedFluidVelocities(materialGrid);
    _updateExtrapolationLayers(materialGrid, layerGrid);

    for (int i = 1; i <= numLayers; i++) {
        _extrapolateVelocitiesForLayerIndex(i, materialGrid, layerGrid);
    }
}