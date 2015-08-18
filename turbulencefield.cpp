#include "turbulencefield.h"


TurbulenceField::TurbulenceField() {
}


TurbulenceField::~TurbulenceField() {
}

void TurbulenceField::_getVelocityGrid(MACVelocityField *macfield, 
                                       Array3d<glm::vec3> &vgrid) {
    glm::vec3 v;
    for (int k = 0; k < vgrid.depth; k++) {
        for (int j = 0; j < vgrid.height; j++) {
            for (int i = 0; i < vgrid.width; i++) {
                v = macfield->evaluateVelocityAtCellCenter(i, j, k);
                vgrid.set(i, j, k, v);
            }
        }
    }
}

double TurbulenceField::_calculateTurbulenceAtGridCell(int i, int j, int k, 
                                                       Array3d<glm::vec3> &vgrid) {
    glm::vec3 vi = vgrid(i, j, k);
    glm::vec3 xi = Grid3d::GridIndexToCellCenter(i, j, k, _dx);
    glm::vec3 vj, vij, vijnorm, xj, xij, xijnorm;
    double vlen, xlen;
    double eps = 10e-6;
    double invradius = 1.0 / _radius;
    double turb = 0.0;

    GridIndex n;
    Grid3d::getNeighbourGridIndices124(i, j, k, cellNeighbours);
    for (int idx = 0; idx < 124; idx++) {
        n = cellNeighbours[idx];
        if (Grid3d::isGridIndexInRange(n.i, n.j, n.k, _isize, _jsize, _ksize)) {

            vj = vgrid(n.i, n.j, n.k);
            vij = vi - vj;
            vlen = glm::length(vij);

            if (fabs(vlen) < eps) {
                continue;
            }
            vijnorm = vij / (float)vlen;

            xj = Grid3d::GridIndexToCellCenter(n.i, n.j, n.k, _dx);
            xij = xi - xj;
            xlen = glm::length(xij);
            xijnorm = xij / (float)xlen;

            turb += vlen*(1.0 - glm::dot(vijnorm, xijnorm))*(1.0 - (xlen*invradius));
        }
    }

    return turb;
}

void TurbulenceField::calculateTurbulenceField(MACVelocityField *vfield,
                                               std::vector<GridIndex> &fluidCells) {
    _field.fill(0.0);

    vfield->getGridDimensions(&_isize, &_jsize, &_ksize);
    _dx = vfield->getGridCellSize();
    _radius = sqrt(3.0*(2*_dx)*(2*_dx));  // maximum distance from center grid cell
                                          // to its 124 neighbours

    if (_field.width != _isize || _field.height != _jsize || _field.depth != _ksize) {
        _field = Array3d<float>(_isize, _jsize, _ksize);
    }

    Array3d<glm::vec3> vgrid = Array3d<glm::vec3>(_isize, _jsize, _ksize);
    _getVelocityGrid(vfield, vgrid);

    double avg = 0.0;
    double max = 0.0;
    double min = std::numeric_limits<double>::infinity();

    GridIndex g;
    for (unsigned int i = 0; i < fluidCells.size(); i++) {
        g = fluidCells[i];
        double t = _calculateTurbulenceAtGridCell(g.i, g.j, g.k, vgrid);
        _field.set(g.i, g.j, g.k, (float)t);

        avg += t;
        if (t > max) {
            max = t;
        }
        if (t < min) {
            min = t;
        }
    }
    avg /= fluidCells.size();

    std::cout << "TFIELD avg: " << avg << " min: " << min << " max: " << max << std::endl;
}

void TurbulenceField::destroyTurbulenceField() {
    _field = Array3d<float>(0, 0, 0);
}

// vertices p are ordered {(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), 
//                         (1, 0, 1), (0, 1, 1), (1, 1, 0), (1, 1, 1)}
// x, y, z, in range [0,1]
double TurbulenceField::_trilinearInterpolate(double p[8], double x, double y, double z) {
    return p[0] * (1 - x) * (1 - y) * (1 - z) +
           p[1] * x * (1 - y) * (1 - z) + 
           p[2] * (1 - x) * y * (1 - z) + 
           p[3] * (1 - x) * (1 - y) * z +
           p[4] * x * (1 - y) * z + 
           p[5] * (1 - x) * y * z + 
           p[6] * x * y * (1 - z) + 
           p[7] * x * y * z;
}

double TurbulenceField::evaluateTurbulenceAtPosition(glm::vec3 p) {
    assert(Grid3d::isPositionInGrid(p, _dx, _isize, _jsize, _ksize));

    p -= glm::vec3(0.5*_dx, 0.5*_dx, 0.5*_dx);

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(p.x, p.y, p.z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (p.x - gx)*inv_dx;
    double iy = (p.y - gy)*inv_dx;
    double iz = (p.z - gz)*inv_dx;

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_field.isIndexInRange(i,   j,   k))   { points[0] = _field(i,   j,   k); }
    if (_field.isIndexInRange(i+1, j,   k))   { points[1] = _field(i+1, j,   k); }
    if (_field.isIndexInRange(i,   j+1, k))   { points[2] = _field(i,   j+1, k); }
    if (_field.isIndexInRange(i,   j,   k+1)) { points[3] = _field(i,   j,   k+1); }
    if (_field.isIndexInRange(i+1, j,   k+1)) { points[4] = _field(i+1, j,   k+1); }
    if (_field.isIndexInRange(i,   j+1, k+1)) { points[5] = _field(i,   j+1, k+1); }
    if (_field.isIndexInRange(i+1, j+1, k))   { points[6] = _field(i+1, j+1, k); }
    if (_field.isIndexInRange(i+1, j+1, k+1)) { points[7] = _field(i+1, j+1, k+1); }

    return _trilinearInterpolate(points, ix, iy, iz);
}