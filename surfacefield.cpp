#include "surfacefield.h"


SurfaceField::SurfaceField()
{
}

SurfaceField::SurfaceField(int w, int h, int d, double cell_size) :
                             i_width(w), j_height(h), k_depth(d), dx(cell_size),
                             width((double)w*cell_size), 
                             height((double)h*cell_size), 
                             depth((double)d*cell_size)
{
}

SurfaceField::~SurfaceField()
{
}

void SurfaceField::setMaterialGrid(Array3d<int> matGrid) {
    materialGrid = matGrid;
    isMaterialGridSet = true;
}

void SurfaceField::setMaterialGrid() {
    materialGrid = Array3d<int>();
    isMaterialGridSet = false;
}

void SurfaceField::clear() {
}

double SurfaceField::getFieldValue(glm::vec3 p) {
    return 0.0;
}

void SurfaceField::positionToGridIndex(glm::vec3 p, int *i, int *j, int *k) {
    double invdx = 1.0 / dx;
    *i = (int)floor(p.x*invdx);
    *j = (int)floor(p.y*invdx);
    *k = (int)floor(p.z*invdx);
}

bool SurfaceField::_isPointNearSolid(glm::vec3 p) {
    double eps = 10e-9;

    glm::vec3 x = glm::vec3(eps, 0.0, 0.0);
    glm::vec3 y = glm::vec3(0.0, eps, 0.0);
    glm::vec3 z = glm::vec3(0.0, 0.0, eps);

    int i, j, k;
    positionToGridIndex(p, &i, &j, &k);
    if (materialGrid(i, j, k) == 2) {
        return true;
    }

    glm::vec3 points[26];
    points[0] = p - x;
    points[1] = p + x;
    points[2] = p - y;
    points[3] = p + y;
    points[4] = p - z;
    points[5] = p + z;
    points[6] = p - x - y;
    points[7] = p - x + y;
    points[8] = p + x - y;
    points[9] = p + x + y;
    points[10] = p - x - z;
    points[11] = p - x + z;
    points[12] = p + x - z;
    points[13] = p + x + z;
    points[14] = p - y - z;
    points[15] = p - y + z;
    points[16] = p + y - z;
    points[17] = p + y + z;
    points[18] = p - x - y - z;
    points[19] = p - x - y + z;
    points[20] = p - x + y - z;
    points[21] = p - x + y + z;
    points[22] = p + x - y - z;
    points[23] = p + x - y + z;
    points[24] = p + x + y - z;
    points[25] = p + x + y + z;
    
    for (int idx = 0; idx < 26; idx++) {
        positionToGridIndex(points[idx], &i, &j, &k);
        if (materialGrid(i, j, k) == 2) {
            return true;
        }
    }

    return false;
}

bool SurfaceField::isInside(glm::vec3 p) {
    return getFieldValue(p) > surfaceThreshold;
}

bool SurfaceField::isOutside(glm::vec3 p) {
    return getFieldValue(p) <= surfaceThreshold;
}
