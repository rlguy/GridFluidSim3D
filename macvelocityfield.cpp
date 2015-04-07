#include "macvelocityfield.h"


MACVelocityField::MACVelocityField()
{
    _initializeVelocityGrids();
}

MACVelocityField::MACVelocityField(int x_voxels, int y_voxels, int z_voxels, double cell_size) :
                                   i_voxels(x_voxels), j_voxels(y_voxels), k_voxels(z_voxels),
                                   dx(cell_size) {

    _initializeVelocityGrids();

}


MACVelocityField::~MACVelocityField()
{
}

void MACVelocityField::_initializeVelocityGrids() {
    _u = new Grid3d(boost::extents[k_voxels][j_voxels][i_voxels + 1]);
    _v = new Grid3d(boost::extents[k_voxels][j_voxels + 1][i_voxels]);
    _w = new Grid3d(boost::extents[k_voxels+ 1][j_voxels][i_voxels]);

    clear();
}

void MACVelocityField::clearU() {
    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels + 1; i++) {
                (*_u)[k][j][i] = 0.0;
            }
        }
    }
}

void MACVelocityField::clearV() {
    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels + 1; j++) {
            for (int i = 0; i < i_voxels; i++) {
                (*_v)[k][j][i] = 0.0;
            }
        }
    }
}

void MACVelocityField::clearW() {
    for (int k = 0; k < k_voxels + 1; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels; i++) {
                (*_w)[k][j][i] = 0.0;
            }
        }
    }
}

void MACVelocityField::clear() {
    clearU();
    clearV();
    clearW();
}

void MACVelocityField::randomizeValues() {
    randomizeValues(-1.0, 1.0);
}

void MACVelocityField::randomizeValues(double min, double max) {
    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels + 1; i++) {
                (*_u)[k][j][i] = _randomFloat(min, max);
            }
        }
    }

    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels + 1; j++) {
            for (int i = 0; i < i_voxels; i++) {
                (*_v)[k][j][i] = _randomFloat(min, max);
            }
        }
    }

    for (int k = 0; k < k_voxels + 1; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels; i++) {
                (*_w)[k][j][i] = _randomFloat(min, max);
            }
        }
    }
}

double MACVelocityField::U(int i, int j, int k) {
    if (!_isIndexInRangeU(i, j, k)) {
        return _default_out_of_range_value;
    }

    return (*_u)[k][j][i];
}

double MACVelocityField::V(int i, int j, int k) {
    if (!_isIndexInRangeV(i, j, k)) {
        return _default_out_of_range_value;
    }

    return (*_v)[k][j][i];
}

double MACVelocityField::W(int i, int j, int k) {
    if (!_isIndexInRangeW(i, j, k)) {
        return _default_out_of_range_value;
    }

    return (*_w)[k][j][i];
}

void MACVelocityField::setU(int i, int j, int k, double val) {
    if (!_isIndexInRangeU(i, j, k)) {
        return;
    }

    (*_u)[k][j][i] = val;
}

void MACVelocityField::setV(int i, int j, int k, double val) {
    if (!_isIndexInRangeV(i, j, k)) {
        return;
    }

    (*_v)[k][j][i] = val;
}

void MACVelocityField::setW(int i, int j, int k, double val) {
    if (!_isIndexInRangeW(i, j, k)) {
        return;
    }

    (*_w)[k][j][i] = val;
}

glm::vec3 MACVelocityField::evaluateVelocityAtCellCenter(int i, int j, int k) {
    assert(_isCellIndexInRange(i, j, k));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i + 1, j, k) + V(i, j, k));
    double zavg = 0.5 * (W(i + 1, j, k) + W(i, j, k));

    return glm::vec3(xavg, yavg, zavg);
}

double MACVelocityField::evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k) {
    assert(_isCellIndexInRange(i, j, k));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i + 1, j, k) + V(i, j, k));
    double zavg = 0.5 * (W(i + 1, j, k) + W(i, j, k));

    return xavg*xavg + yavg*yavg + zavg*zavg;
}

double MACVelocityField::evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k) {
    assert(_isCellIndexInRange(i, j, k));

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
    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels; i++) {
                
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
    assert(_isIndexInRangeU(i, j, k));

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
    assert(_isIndexInRangeV(i, j, k));

    j--;

    double vx = 0.25 * (U(i, j, k) + U(i+1, j, k) + U(i, j+1, k) + U(i+1, j+1, k));
    double vy = V(i, j + 1, k);
    double vz = 0.25 * (W(i, j, k) + W(i, j, k+1) + W(i, j+1, k) + W(i, j+1, k+1));

    return glm::vec3(vx, vy, vz);
}

glm::vec3 MACVelocityField::evaluateVelocityAtFaceCenterW(int i, int j, int k) {
    assert(_isIndexInRangeW(i, j, k));

    k--;

    double vx = 0.25 * (U(i, j, k) + U(i+1, j, k) + U(i, j, k+1) + U(i+1, j, k+1));
    double vy = 0.25 * (V(i, j, k) + V(i, j+1, k) + V(i, j, k+1) + V(i, j+1, k+1));
    double vz = W(i, j, k + 1);

    return glm::vec3(vx, vy, vz);
}

/* 
    Intopolation methods from http://www.paulinternet.nl/?page=bicubic

    - p is indexed in order by p[k][j][i]
    - x, y, z are in [0,1]
    - this function will interpolate the volume between point Index 1 and 2
*/
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
    assert(_isPositionInGrid(x, y, z));

    // Velocity positions are staggered from grid cell positions. We need to find the minimal
    // U coordinate (of the 8 that form a cube) that encloses point (x, y, z) so that we have
    // a reference to construct a 4x4x4 array of U values for a tricubic interpolation
    int i, j, k;
    double gx, gy, gz;
    _positionToGridIndex(x, y, z, &i, &j, &k);
    _gridIndexToPosition(i, j, k, &gx, &gy, &gz);

    double refx = gx;
    double refy = gy - 0.5*dx;
    double refz = gz - 0.5*dx;
    int refi = i;
    int refj = j - 1;
    int refk = k - 1;

    double cy = gy + 0.5*dx;
    double cz = gz + 0.5*dx;

    if (y > cy) {
        refj++;
        refy += dx;
    }

    if (z > cz) {
        refk++;
        refz += dx;
    }

    double invdx = 1 / dx;
    double ix = (x - refx)*dx;
    double iy = (y - refy)*dx;
    double iz = (z - refz)*dx;

    assert(ix >= 0 && ix <= 1 && iy >= 0 && iy <= 1 && iz >= 0 && iz <= 1);

    refi--; refj--; refk--;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                points[pk][pj][pi] = U(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateV(double x, double y, double z) {
    assert(_isPositionInGrid(x, y, z));

    int i, j, k;
    double gx, gy, gz;
    _positionToGridIndex(x, y, z, &i, &j, &k);
    _gridIndexToPosition(i, j, k, &gx, &gy, &gz);

    double refx = gx - 0.5*dx;
    double refy = gy;
    double refz = gz - 0.5*dx;
    int refi = i - 1;
    int refj = j;
    int refk = k - 1;

    double cx = gx + 0.5*dx;
    double cz = gz + 0.5*dx;

    if (x > cx) {
        refi++;
        refx += dx;
    }

    if (z > cz) {
        refk++;
        refz += dx;
    }

    double invdx = 1 / dx;
    double ix = (x - refx)*dx;
    double iy = (y - refy)*dx;
    double iz = (z - refz)*dx;

    assert(ix >= 0 && ix <= 1 && iy >= 0 && iy <= 1 && iz >= 0 && iz <= 1);

    refi--; refj--; refk--;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                points[pk][pj][pi] = U(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateW(double x, double y, double z) {
    assert(_isPositionInGrid(x, y, z));

    int i, j, k;
    double gx, gy, gz;
    _positionToGridIndex(x, y, z, &i, &j, &k);
    _gridIndexToPosition(i, j, k, &gx, &gy, &gz);

    double refx = gx - 0.5*dx;
    double refy = gy - 0.5*dx;
    double refz = gz;
    int refi = i - 1;
    int refj = j - 1;
    int refk = k;

    double cx = gx + 0.5*dx;
    double cy = gy + 0.5*dx;

    if (x > cx) {
        refi++;
        refx += dx;
    }

    if (y > cy) {
        refj++;
        refy += dx;
    }

    double invdx = 1 / dx;
    double ix = (x - refx)*dx;
    double iy = (y - refy)*dx;
    double iz = (z - refz)*dx;

    assert(ix >= 0 && ix <= 1 && iy >= 0 && iy <= 1 && iz >= 0 && iz <= 1);

    refi--; refj--; refk--;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                points[pk][pj][pi] = U(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return _tricubicInterpolate(points, ix, iy, iz);
}

void MACVelocityField::_positionToGridIndex(double x, double y, double z, int *i, int *j, int *k) {
    assert(_isPositionInGrid(x, y, z));

    double invdx = 1.0 / dx;
    *i = (int)floor(x*invdx);
    *j = (int)floor(y*invdx);
    *k = (int)floor(z*invdx);
}

void MACVelocityField::_gridIndexToPosition(int i, int j, int k, double *x, double *y, double *z) {
    assert(_isCellIndexInRange(i, j, k));

    *x = (double)i*dx;
    *y = (double)j*dx;
    *z = (double)k*dx;
}

glm::vec3 MACVelocityField::evaluateVelocityAtPosition(glm::vec3 pos) {
    return evaluateVelocityAtPosition(pos.x, pos.x, pos.z);
}

glm::vec3 MACVelocityField::evaluateVelocityAtPosition(double x, double y, double z) {
    assert(_isPositionInGrid(x, y, z));

    double xvel = _interpolateU(x, y, z);
    double yvel = _interpolateV(x, y, z);
    double zvel = _interpolateW(x, y, z);

    return glm::vec3(xvel, yvel, zvel);
}