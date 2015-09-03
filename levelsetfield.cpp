#include "levelsetfield.h"


LevelSetField::LevelSetField(int i, int j, int k, double dx) :
                             SurfaceField(i, j, k, dx),
                             _distanceField(Array3d<float>(i, j, k, 1.0f))
{
    setSurfaceThreshold(_surfaceThreshold);
}

LevelSetField::LevelSetField()
{
}

LevelSetField::~LevelSetField()
{
}

void LevelSetField::clear() {
    _distanceField.fill(0.0);
}

double LevelSetField::getFieldValue(glm::vec3 p) {
    GridIndex g = Grid3d::positionToGridIndex(p, dx);
    glm::vec3 gp = Grid3d::GridIndexToPosition(g, dx);

    double refx = gp.x - 0.5*dx;
    double refy = gp.y - 0.5*dx;
    double refz = gp.z - 0.5*dx;
    int refi = g.i - 1; 
    int refj = g.j - 1; 
    int refk = g.k - 1;

    glm::vec3 offset = p - gp;
    if (offset.x >= 0.5*dx) {
        refi++;
        refx += dx;
    }
    if (offset.y >= 0.5*dx) {
        refj++;
        refy += dx;
    }
    if (offset.z >= 0.5*dx) {
        refk++;
        refz += dx;
    }

    double invdx = 1 / dx;
    double ix = (p.x - refx)*invdx;
    double iy = (p.y - refy)*invdx;
    double iz = (p.z - refz)*invdx;

    assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    g = GridIndex(refi - 1, refj - 1, refk - 1);
    double points[4][4][4];
    GridIndex c;
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                c = GridIndex(pi + g.i, pj + g.j, pk + g.k);
                if (Grid3d::isGridIndexInRange(c, i_width, j_height, k_depth)) {
                    points[pi][pj][pk] = _distanceField(c);
                } else {
                    points[pi][pj][pk] = 0.0;
                }
            }
        }
    }

    double val = _tricubicInterpolate(points, ix, iy, iz);

    double eps = 10e-6;
    if (isMaterialGridSet && val > surfaceThreshold && _isPointNearSolid(p)) {
        val = surfaceThreshold - eps;
    }

    return val;
}

void LevelSetField::setSignedDistanceField(Array3d<float> distField) {
    _distanceField = distField;
}

// vertices p are ordered {(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), 
//                         (1, 0, 1), (0, 1, 1), (1, 1, 0), (1, 1, 1)}
// x, y, z, in range [0,1]
double LevelSetField::_trilinearInterpolate(double p[8], double x, double y, double z) {
    return p[0] * (1 - x) * (1 - y) * (1 - z) +
           p[1] * x * (1 - y) * (1 - z) + 
           p[2] * (1 - x) * y * (1 - z) + 
           p[3] * (1 - x) * (1 - y) * z +
           p[4] * x * (1 - y) * z + 
           p[5] * (1 - x) * y * z + 
           p[6] * x * y * (1 - z) + 
           p[7] * x * y * z;
}

double LevelSetField::_tricubicInterpolate(double p[4][4][4], double x, double y, double z) {
    assert(x >= 0 && x <= 1 && y >= 0 && y <= 1 && z >= 0 && z <= 1);

    double arr[4];
    arr[0] = _bicubicInterpolate(p[0], y, z);
    arr[1] = _bicubicInterpolate(p[1], y, z);
    arr[2] = _bicubicInterpolate(p[2], y, z);
    arr[3] = _bicubicInterpolate(p[3], y, z);
    return _cubicInterpolate(arr, x);
}

double LevelSetField::_bicubicInterpolate(double p[4][4], double x, double y) {
    double arr[4];
    arr[0] = _cubicInterpolate(p[0], y);
    arr[1] = _cubicInterpolate(p[1], y);
    arr[2] = _cubicInterpolate(p[2], y);
    arr[3] = _cubicInterpolate(p[3], y);
    return _cubicInterpolate(arr, x);
}

double LevelSetField::_cubicInterpolate(double p[4], double x) {
    return p[1] + 0.5 * x*(p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0])));
}