#include "macvelocityfield.h"


MACVelocityField::MACVelocityField()
{
    _initializeVelocityGrids();
}

MACVelocityField::MACVelocityField(int x_voxels, int y_voxels, int z_voxels, double cell_size) :
                                   _isize(x_voxels), _jsize(y_voxels), _ksize(z_voxels),
                                   _dx(cell_size) {

    _initializeVelocityGrids();
}


MACVelocityField::~MACVelocityField()
{

}

void MACVelocityField::_initializeVelocityGrids() {

    _u = Array3d<float>(_isize + 1, _jsize, _ksize, 0.0f);
    _v = Array3d<float>(_isize, _jsize + 1, _ksize, 0.0f);
    _w = Array3d<float>(_isize, _jsize, _ksize + 1, 0.0f);

    _save_u = Array3d<float>(_isize + 1, _jsize, _ksize, 0.0f);
    _save_v = Array3d<float>(_isize, _jsize + 1, _ksize, 0.0f);
    _save_w = Array3d<float>(_isize, _jsize, _ksize + 1, 0.0f);

    _temp_u = Array3d<float>(_isize + 1, _jsize, _ksize, 0.0f);
    _temp_v = Array3d<float>(_isize, _jsize + 1, _ksize, 0.0f);
    _temp_w = Array3d<float>(_isize, _jsize, _ksize + 1, 0.0f);

    _is_set_u = Array3d<bool>(_isize + 1, _jsize, _ksize, false);
    _is_set_v = Array3d<bool>(_isize, _jsize + 1, _ksize, false);
    _is_set_w = Array3d<bool>(_isize, _jsize, _ksize + 1, false);
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

void MACVelocityField::resetTemporaryVelocityField() {
    _temp_u.fill(0.0);
    _temp_v.fill(0.0);
    _temp_w.fill(0.0);
    _is_set_u.fill(false);
    _is_set_v.fill(false);
    _is_set_w.fill(false);
}

void MACVelocityField::saveVelocityFieldState() {
    _save_u = _u;
    _save_v = _v;
    _save_w = _w;
}

void MACVelocityField::commitTemporaryVelocityFieldValues() {
    for (int k = 0; k < _is_set_u.depth; k++) {
        for (int j = 0; j < _is_set_u.height; j++) {
            for (int i = 0; i < _is_set_u.width; i++) {
                if (_is_set_u(i, j, k)) {
                    setU(i, j, k, _temp_u(i, j, k));
                }
            }
        }
    }

    for (int k = 0; k < _is_set_v.depth; k++) {
        for (int j = 0; j < _is_set_v.height; j++) {
            for (int i = 0; i < _is_set_v.width; i++) {
                if (_is_set_v(i, j, k)) {
                    setV(i, j, k, _temp_v(i, j, k));
                }
            }
        }
    }

    for (int k = 0; k < _is_set_w.depth; k++) {
        for (int j = 0; j < _is_set_w.height; j++) {
            for (int i = 0; i < _is_set_w.width; i++) {
                if (_is_set_w(i, j, k)) {
                    setW(i, j, k, _temp_w(i, j, k));
                }
            }
        }
    }
}

void MACVelocityField::randomizeValues() {
    randomizeValues(-1.0, 1.0);
}

void MACVelocityField::randomizeValues(double min, double max) {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                _u.set(i, j, k, _randomFloat(min, max));
            }
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                _v.set(i, j, k, _randomFloat(min, max));
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                _w.set(i, j, k, _randomFloat(min, max));
            }
        }
    }
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

double MACVelocityField::U(int i, int j, int k) {
    if (!isIndexInRangeU(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _u(i, j, k);
}

double MACVelocityField::V(int i, int j, int k) {
    if (!isIndexInRangeV(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _v(i, j, k);
}

double MACVelocityField::W(int i, int j, int k) {
    if (!isIndexInRangeW(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _w(i, j, k);
}

double MACVelocityField::U(GridIndex g) {
    if (!isIndexInRangeU(g)) {
        return _default_out_of_range_value;
    }

    return _u(g);
}

double MACVelocityField::V(GridIndex g) {
    if (!isIndexInRangeV(g)) {
        return _default_out_of_range_value;
    }

    return _v(g);
}

double MACVelocityField::W(GridIndex g) {
    if (!isIndexInRangeW(g)) {
        return _default_out_of_range_value;
    }

    return _w(g);
}

double MACVelocityField::tempU(int i, int j, int k) {
    if (!isIndexInRangeU(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _temp_u(i, j, k);
}

double MACVelocityField::tempV(int i, int j, int k) {
    if (!isIndexInRangeV(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _temp_v(i, j, k);
}

double MACVelocityField::tempW(int i, int j, int k) {
    if (!isIndexInRangeW(i, j, k)) {
        return _default_out_of_range_value;
    }

    return _temp_w(i, j, k);
}

void MACVelocityField::setU(int i, int j, int k, double val) {
    if (!isIndexInRangeU(i, j, k)) {
        return;
    }

    _u.set(i, j, k, val);
}

void MACVelocityField::setV(int i, int j, int k, double val) {
    if (!isIndexInRangeV(i, j, k)) {
        return;
    }

    _v.set(i, j, k, val);
}

void MACVelocityField::setW(int i, int j, int k, double val) {
    if (!isIndexInRangeW(i, j, k)) {
        return;
    }

    _w.set(i, j, k, val);
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
    assert(ugrid.width == _u.width && 
           ugrid.height == _u.height && 
           ugrid.depth == _u.depth);
    _u = ugrid;
}

void MACVelocityField::setV(Array3d<float> &vgrid) {
    assert(vgrid.width == _v.width && 
           vgrid.height == _v.height && 
           vgrid.depth == _v.depth);
    _v = vgrid;
}

void MACVelocityField::setW(Array3d<float> &wgrid) {
    assert(wgrid.width == _w.width && 
           wgrid.height == _w.height && 
           wgrid.depth == _w.depth);
    _w = wgrid;
}

void MACVelocityField::addU(int i, int j, int k, double val) {
    if (!isIndexInRangeU(i, j, k)) {
        return;
    }

    _u.add(i, j, k, val);
}

void MACVelocityField::addV(int i, int j, int k, double val) {
    if (!isIndexInRangeV(i, j, k)) {
        return;
    }

    _v.add(i, j, k, val);
}

void MACVelocityField::addW(int i, int j, int k, double val) {
    if (!isIndexInRangeW(i, j, k)) {
        return;
    }

    _w.add(i, j, k, val);
}

void MACVelocityField::setTempU(int i, int j, int k, double val) {
    if (!isIndexInRangeU(i, j, k)) {
        return;
    }

    _temp_u.set(i, j, k, val);
    _is_set_u.set(i, j, k, true);
}

void MACVelocityField::setTempV(int i, int j, int k, double val) {
    if (!isIndexInRangeV(i, j, k)) {
        return;
    }

    _temp_v.set(i, j, k, val);
    _is_set_v.set(i, j, k, true);
}

void MACVelocityField::setTempW(int i, int j, int k, double val) {
    if (!isIndexInRangeW(i, j, k)) {
        return;
    }

    _temp_w.set(i, j, k, val);
    _is_set_w.set(i, j, k, true);
}

void MACVelocityField::addTempU(int i, int j, int k, double val) {
    if (!isIndexInRangeU(i, j, k)) {
        return;
    }

    _temp_u.add(i, j, k, val);
    _is_set_u.set(i, j, k, true);
}

void MACVelocityField::addTempV(int i, int j, int k, double val) {
    if (!isIndexInRangeV(i, j, k)) {
        return;
    }

    _temp_v.add(i, j, k, val);
    _is_set_v.set(i, j, k, true);
}

void MACVelocityField::addTempW(int i, int j, int k, double val) {
    if (!isIndexInRangeW(i, j, k)) {
        return;
    }

    _temp_w.add(i, j, k, val);
    _is_set_w.set(i, j, k, true);
}

glm::vec3 MACVelocityField::velocityIndexToPositionU(int i, int j, int k) {
    assert(isIndexInRangeU(i, j, k));

    double gx = (double)(i-1)*_dx;
    double gy = (double)j*_dx;
    double gz = (double)k*_dx;

    return glm::vec3(gx + _dx, gy + 0.5*_dx, gz + 0.5*_dx);
}

glm::vec3 MACVelocityField::velocityIndexToPositionV(int i, int j, int k) {
    assert(isIndexInRangeV(i, j, k));

    double gx = (double)i*_dx;
    double gy = (double)(j-1)*_dx;
    double gz = (double)k*_dx;

    return glm::vec3(gx + 0.5*_dx, gy + _dx, gz + 0.5*_dx);
}

glm::vec3 MACVelocityField::velocityIndexToPositionW(int i, int j, int k) {
    assert(isIndexInRangeW(i, j, k));

    double gx = (double)i*_dx;
    double gy = (double)j*_dx;
    double gz = (double)(k-1)*_dx;

    return glm::vec3(gx + 0.5*_dx, gy + 0.5*_dx, gz + _dx);
}

glm::vec3 MACVelocityField::evaluateVelocityAtCellCenter(int i, int j, int k) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i, j + 1, k) + V(i, j, k));
    double zavg = 0.5 * (W(i, j, k + 1) + W(i, j, k));

    return glm::vec3(xavg, yavg, zavg);
}

double MACVelocityField::evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i, j + 1, k) + V(i, j, k));
    double zavg = 0.5 * (W(i, j, k + 1) + W(i, j, k));

    return xavg*xavg + yavg*yavg + zavg*zavg;
}

double MACVelocityField::evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double mag = evaluateVelocityMagnitudeSquaredAtCellCenter(i, j, k);
    if (mag > 0.0) {
        return sqrt(mag);
    }
    else {
        return 0.0;
    }
}

double MACVelocityField::evaluateMaximumVelocityMagnitude() {
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

    return max;
}

glm::vec3 MACVelocityField::evaluateVelocityAtFaceCenterU(int i, int j, int k) {
    assert(isIndexInRangeU(i, j, k));

    // Shift reference coordinate to the left. The formula used is for calculating
    // u(i+1/2, j, k). If we keep original (i,j,k) coordinate, then using the formula
    // would calculate u(i+3/2, j, k) instead. The same will be done for the V and W
    // faces, shifting back in the respective direction.
    i--;

    double vx = U(i+1, j, k);
    double vy = 0.25 * (V(i, j, k) + V(i, j+1, k) + V(i+1, j, k) + V(i+1, j+1, k));
    double vz = 0.25 * (W(i, j, k) + W(i, j, k+1) + W(i+1, j, k) + W(i+1, j, k+1));

    return glm::vec3(vx, vy, vz);
}

glm::vec3 MACVelocityField::evaluateVelocityAtFaceCenterV(int i, int j, int k) {
    assert(isIndexInRangeV(i, j, k));

    j--;

    double vx = 0.25 * (U(i, j, k) + U(i+1, j, k) + U(i, j+1, k) + U(i+1, j+1, k));
    double vy = V(i, j + 1, k);
    double vz = 0.25 * (W(i, j, k) + W(i, j, k+1) + W(i, j+1, k) + W(i, j+1, k+1));

    return glm::vec3(vx, vy, vz);
}

glm::vec3 MACVelocityField::evaluateVelocityAtFaceCenterW(int i, int j, int k) {
    assert(isIndexInRangeW(i, j, k));

    k--;

    double vx = 0.25 * (U(i, j, k) + U(i+1, j, k) + U(i, j, k+1) + U(i+1, j, k+1));
    double vy = 0.25 * (V(i, j, k) + V(i, j+1, k) + V(i, j, k+1) + V(i, j+1, k+1));
    double vz = W(i, j, k + 1);

    return glm::vec3(vx, vy, vz);
}

// vertices p are ordered {(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), 
//                         (1, 0, 1), (0, 1, 1), (1, 1, 0), (1, 1, 1)}
// x, y, z, in range [0,1]
double MACVelocityField::_trilinearInterpolate(double p[8], double x, double y, double z) {
    return p[0] * (1 - x) * (1 - y) * (1 - z) +
           p[1] * x * (1 - y) * (1 - z) + 
           p[2] * (1 - x) * y * (1 - z) + 
           p[3] * (1 - x) * (1 - y) * z +
           p[4] * x * (1 - y) * z + 
           p[5] * (1 - x) * y * z + 
           p[6] * x * y * (1 - z) + 
           p[7] * x * y * z;
}

double MACVelocityField::_tricubicInterpolate(double p[4][4][4], double x, double y, double z) {
    assert(x >= 0 && x <= 1 && y >= 0 && y <= 1 && z >= 0 && z <= 1);

    double arr[4];
    arr[0] = _bicubicInterpolate(p[0], y, z);
    arr[1] = _bicubicInterpolate(p[1], y, z);
    arr[2] = _bicubicInterpolate(p[2], y, z);
    arr[3] = _bicubicInterpolate(p[3], y, z);
    return _cubicInterpolate(arr, x);
}

double MACVelocityField::_bicubicInterpolate(double p[4][4], double x, double y) {
    double arr[4];
    arr[0] = _cubicInterpolate(p[0], y);
    arr[1] = _cubicInterpolate(p[1], y);
    arr[2] = _cubicInterpolate(p[2], y);
    arr[3] = _cubicInterpolate(p[3], y);
    return _cubicInterpolate(arr, x);
}

double MACVelocityField::_cubicInterpolate(double p[4], double x) {
    return p[1] + 0.5 * x*(p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0])));
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                if (_u.isIndexInRange(pi + refi, pj + refj, pk + refk)) {
                    points[pi][pj][pk] = U(pi + refi, pj + refj, pk + refk);
                } else {
                    points[pi][pj][pk] = 0.0;
                }
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                if (_v.isIndexInRange(pi + refi, pj + refj, pk + refk)) {
                    points[pi][pj][pk] = V(pi + refi, pj + refj, pk + refk);
                } else {
                    points[pi][pj][pk] = 0;
                }
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                if (_w.isIndexInRange(pi + refi, pj + refj, pk + refk)) {
                    points[pi][pj][pk] = W(pi + refi, pj + refj, pk + refk);
                } else {
                    points[pi][pj][pk] = 0;
                }
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_u.isIndexInRange(i,   j,   k))   { points[0] = _u(i,   j,   k); }
    if (_u.isIndexInRange(i+1, j,   k))   { points[1] = _u(i+1, j,   k); }
    if (_u.isIndexInRange(i,   j+1, k))   { points[2] = _u(i,   j+1, k); }
    if (_u.isIndexInRange(i,   j,   k+1)) { points[3] = _u(i,   j,   k+1); }
    if (_u.isIndexInRange(i+1, j,   k+1)) { points[4] = _u(i+1, j,   k+1); }
    if (_u.isIndexInRange(i,   j+1, k+1)) { points[5] = _u(i,   j+1, k+1); }
    if (_u.isIndexInRange(i+1, j+1, k))   { points[6] = _u(i+1, j+1, k); }
    if (_u.isIndexInRange(i+1, j+1, k+1)) { points[7] = _u(i+1, j+1, k+1); }

    return _trilinearInterpolate(points, ix, iy, iz);
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_v.isIndexInRange(i,   j,   k))   { points[0] = _v(i,   j,   k); }
    if (_v.isIndexInRange(i+1, j,   k))   { points[1] = _v(i+1, j,   k); }
    if (_v.isIndexInRange(i,   j+1, k))   { points[2] = _v(i,   j+1, k); }
    if (_v.isIndexInRange(i,   j,   k+1)) { points[3] = _v(i,   j,   k+1); }
    if (_v.isIndexInRange(i+1, j,   k+1)) { points[4] = _v(i+1, j,   k+1); }
    if (_v.isIndexInRange(i,   j+1, k+1)) { points[5] = _v(i,   j+1, k+1); }
    if (_v.isIndexInRange(i+1, j+1, k))   { points[6] = _v(i+1, j+1, k); }
    if (_v.isIndexInRange(i+1, j+1, k+1)) { points[7] = _v(i+1, j+1, k+1); }

    return _trilinearInterpolate(points, ix, iy, iz);
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_w.isIndexInRange(i,   j,   k))   { points[0] = _w(i,   j,   k); }
    if (_w.isIndexInRange(i+1, j,   k))   { points[1] = _w(i+1, j,   k); }
    if (_w.isIndexInRange(i,   j+1, k))   { points[2] = _w(i,   j+1, k); }
    if (_w.isIndexInRange(i,   j,   k+1)) { points[3] = _w(i,   j,   k+1); }
    if (_w.isIndexInRange(i+1, j,   k+1)) { points[4] = _w(i+1, j,   k+1); }
    if (_w.isIndexInRange(i,   j+1, k+1)) { points[5] = _w(i,   j+1, k+1); }
    if (_w.isIndexInRange(i+1, j+1, k))   { points[6] = _w(i+1, j+1, k); }
    if (_w.isIndexInRange(i+1, j+1, k+1)) { points[7] = _w(i+1, j+1, k+1); }

    return _trilinearInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateDeltaVelocityU(double x, double y, double z) {
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    /*
    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_u.isIndexInRange(i,   j,   k))   { points[0] = _u(i,   j,   k)   - _save_u(i,   j,   k); }
    if (_u.isIndexInRange(i+1, j,   k))   { points[1] = _u(i+1, j,   k)   - _save_u(i+1, j,   k); }
    if (_u.isIndexInRange(i,   j+1, k))   { points[2] = _u(i,   j+1, k)   - _save_u(i,   j+1, k); }
    if (_u.isIndexInRange(i,   j,   k+1)) { points[3] = _u(i,   j,   k+1) - _save_u(i,   j,   k+1); }
    if (_u.isIndexInRange(i+1, j,   k+1)) { points[4] = _u(i+1, j,   k+1) - _save_u(i+1, j,   k+1); }
    if (_u.isIndexInRange(i,   j+1, k+1)) { points[5] = _u(i,   j+1, k+1) - _save_u(i,   j+1, k+1); }
    if (_u.isIndexInRange(i+1, j+1, k))   { points[6] = _u(i+1, j+1, k)   - _save_u(i+1, j+1, k); }
    if (_u.isIndexInRange(i+1, j+1, k+1)) { points[7] = _u(i+1, j+1, k+1) - _save_u(i+1, j+1, k+1); }
    */

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                if (_u.isIndexInRange(pi + refi, pj + refj, pk + refk)) {
                    points[pi][pj][pk] = U(pi + refi, pj + refj, pk + refk) - 
                                         _save_u(pi + refi, pj + refj, pk + refk);
                } else {
                    points[pi][pj][pk] = 0;
                }
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateDeltaVelocityV(double x, double y, double z) {
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    /*
    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_v.isIndexInRange(i,   j,   k))   { points[0] = _v(i,   j,   k)   - _save_v(i,   j,   k); }
    if (_v.isIndexInRange(i+1, j,   k))   { points[1] = _v(i+1, j,   k)   - _save_v(i+1, j,   k); }
    if (_v.isIndexInRange(i,   j+1, k))   { points[2] = _v(i,   j+1, k)   - _save_v(i,   j+1, k); }
    if (_v.isIndexInRange(i,   j,   k+1)) { points[3] = _v(i,   j,   k+1) - _save_v(i,   j,   k+1); }
    if (_v.isIndexInRange(i+1, j,   k+1)) { points[4] = _v(i+1, j,   k+1) - _save_v(i+1, j,   k+1); }
    if (_v.isIndexInRange(i,   j+1, k+1)) { points[5] = _v(i,   j+1, k+1) - _save_v(i,   j+1, k+1); }
    if (_v.isIndexInRange(i+1, j+1, k))   { points[6] = _v(i+1, j+1, k)   - _save_v(i+1, j+1, k); }
    if (_v.isIndexInRange(i+1, j+1, k+1)) { points[7] = _v(i+1, j+1, k+1) - _save_v(i+1, j+1, k+1); }
    */

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                if (_v.isIndexInRange(pi + refi, pj + refj, pk + refk)) {
                    points[pi][pj][pk] = V(pi + refi, pj + refj, pk + refk) - 
                                         _save_v(pi + refi, pj + refj, pk + refk);
                } else {
                    points[pi][pj][pk] = 0;
                }
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateDeltaVelocityW(double x, double y, double z) {
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

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    /*
    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_w.isIndexInRange(i,   j,   k))   { points[0] = _w(i,   j,   k)   - _save_w(i,   j,   k); }
    if (_w.isIndexInRange(i+1, j,   k))   { points[1] = _w(i+1, j,   k)   - _save_w(i+1, j,   k); }
    if (_w.isIndexInRange(i,   j+1, k))   { points[2] = _w(i,   j+1, k)   - _save_w(i,   j+1, k); }
    if (_w.isIndexInRange(i,   j,   k+1)) { points[3] = _w(i,   j,   k+1) - _save_w(i,   j,   k+1); }
    if (_w.isIndexInRange(i+1, j,   k+1)) { points[4] = _w(i+1, j,   k+1) - _save_w(i+1, j,   k+1); }
    if (_w.isIndexInRange(i,   j+1, k+1)) { points[5] = _w(i,   j+1, k+1) - _save_w(i,   j+1, k+1); }
    if (_w.isIndexInRange(i+1, j+1, k))   { points[6] = _w(i+1, j+1, k)   - _save_w(i+1, j+1, k); }
    if (_w.isIndexInRange(i+1, j+1, k+1)) { points[7] = _w(i+1, j+1, k+1) - _save_w(i+1, j+1, k+1); }
    */

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                if (_w.isIndexInRange(pi + refi, pj + refj, pk + refk)) {
                    points[pi][pj][pk] = W(pi + refi, pj + refj, pk + refk) - 
                                         _save_w(pi + refi, pj + refj, pk + refk);
                } else {
                    points[pi][pj][pk] = 0;
                }
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
}


glm::vec3 MACVelocityField::evaluateVelocityAtPosition(glm::vec3 pos) {
    return evaluateVelocityAtPosition(pos.x, pos.y, pos.z);
}

glm::vec3 MACVelocityField::evaluateVelocityAtPosition(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return glm::vec3(0.0, 0.0, 0.0);
    }

    double xvel = _interpolateU(x, y, z);
    double yvel = _interpolateV(x, y, z);
    double zvel = _interpolateW(x, y, z);

    return glm::vec3(xvel, yvel, zvel);
}

glm::vec3 MACVelocityField::evaluateVelocityAtPositionLinear(glm::vec3 pos) {
    return evaluateVelocityAtPositionLinear(pos.x, pos.y, pos.z);
}

glm::vec3 MACVelocityField::evaluateVelocityAtPositionLinear(double x, double y, double z) {
    if (!Grid3d::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize)) {
        return glm::vec3(0.0, 0.0, 0.0);
    }

    double xvel = _interpolateLinearU(x, y, z);
    double yvel = _interpolateLinearV(x, y, z);
    double zvel = _interpolateLinearW(x, y, z);

    return glm::vec3(xvel, yvel, zvel);
}

glm::vec3 MACVelocityField::evaluateChangeInVelocityAtPosition(glm::vec3 p) {
    if (!Grid3d::isPositionInGrid(p.x, p.y, p.z, _dx, _isize, _jsize, _ksize)) {
        return glm::vec3(0.0, 0.0, 0.0);
    }

    double xvel = _interpolateDeltaVelocityU(p.x, p.y, p.z);
    double yvel = _interpolateDeltaVelocityV(p.x, p.y, p.z);
    double zvel = _interpolateDeltaVelocityW(p.x, p.y, p.z);

    return glm::vec3(xvel, yvel, zvel);
}