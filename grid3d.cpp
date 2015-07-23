#include "grid3d.h"

void Grid3d::positionToGridIndex(double x, double y, double z, double dx,
                                 int *i, int *j, int *k) {
    double invdx = 1.0 / dx;
    *i = (int)floor(x*invdx);
    *j = (int)floor(y*invdx);
    *k = (int)floor(z*invdx);
}

void Grid3d::positionToGridIndex(glm::vec3 p, double dx,
                                 int *i, int *j, int *k) {
    double invdx = 1.0 / dx;
    *i = (int)floor(p.x*invdx);
    *j = (int)floor(p.y*invdx);
    *k = (int)floor(p.z*invdx);
}

GridIndex Grid3d::positionToGridIndex(double x, double y, double z, double dx) {
    double invdx = 1.0 / dx;
    return GridIndex((int)floor(x*invdx),
                     (int)floor(y*invdx),
                     (int)floor(z*invdx));
}
    
GridIndex Grid3d::positionToGridIndex(glm::vec3 p, double dx) {
    double invdx = 1.0 / dx;
    return GridIndex((int)floor(p.x*invdx),
                     (int)floor(p.y*invdx),
                     (int)floor(p.z*invdx));
}

void Grid3d::GridIndexToPosition(int i, int j, int k, double dx, 
                                 double *x, double *y, double *z) {
    *x = (double)i*dx;
    *y = (double)j*dx;
    *z = (double)k*dx;
}

void Grid3d::GridIndexToPosition(GridIndex g, double dx, 
                                 double *x, double *y, double *z) {
    *x = (double)g.i*dx;
    *y = (double)g.j*dx;
    *z = (double)g.k*dx;
}

glm::vec3 Grid3d::GridIndexToPosition(int i, int j, int k, double dx) {
    return glm::vec3((float)i*dx, (float)j*dx, (float)k*dx);
}

glm::vec3 Grid3d::GridIndexToPosition(GridIndex g, double dx) {
    return glm::vec3((float)g.i*dx, (float)g.j*dx, (float)g.k*dx);
}

void Grid3d::GridIndexToCellCenter(int i, int j, int k, double dx, 
                                   double *x, double *y, double *z) {
    double hw = 0.5*dx;
    *x = (double)i*dx + hw;
    *y = (double)j*dx + hw;
    *z = (double)k*dx + hw;
}

void Grid3d::GridIndexToCellCenter(GridIndex g, double dx, 
                                   double *x, double *y, double *z) {
    double hw = 0.5*dx;
    *x = (double)g.i*dx + hw;
    *y = (double)g.j*dx + hw;
    *z = (double)g.k*dx + hw;
}

glm::vec3 Grid3d::GridIndexToCellCenter(int i, int j, int k, double dx) {
    double hw = 0.5*dx;
    return glm::vec3((float)i*dx + hw, (float)j*dx + hw, (float)k*dx + hw);
}

glm::vec3 Grid3d::GridIndexToCellCenter(GridIndex g, double dx) {
    double hw = 0.5*dx;
    return glm::vec3((float)g.i*dx + hw, (float)g.j*dx + hw, (float)g.k*dx + hw);
}

bool Grid3d::isPositionInGrid(double x, double y, double z, double dx, int i, int j, int k) {
    return x >= 0 && y >= 0 && z >= 0 && x < dx*i && y < dx*j && z < dx*k;
}

bool Grid3d::isPositionInGrid(glm::vec3 p, double dx, int i, int j, int k) {
    return p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < dx*i && p.y < dx*j && p.z < dx*k;
}

bool Grid3d::isPositionInGrid(double x, double y, double z, double dx, GridIndex g) {
    return x >= 0 && y >= 0 && z >= 0 && x < dx*g.i && y < dx*g.j && z < dx*g.k;
}

bool Grid3d::isPositionInGrid(glm::vec3 p, double dx, GridIndex g) {
    return p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < dx*g.i && p.y < dx*g.j && p.z < dx*g.k;
}

bool Grid3d::isGridIndexInRange(int i, int j, int k, int imax, int jmax, int kmax) {
    return i >= 0 && j >= 0 && k >= 0 && i < imax && j < jmax && k < kmax;
}

bool Grid3d::isGridIndexInRange(GridIndex g, int imax, int jmax, int kmax) {
    return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < imax && g.j < jmax && g.k < kmax;
}

bool Grid3d::isGridIndexInRange(int i, int j, int k, GridIndex gmax) {
    return i >= 0 && j >= 0 && k >= 0 && i < gmax.i && j < gmax.j && k < gmax.k;
}

bool Grid3d::isGridIndexInRange(GridIndex g, GridIndex gmax) {
    return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < gmax.i && g.j < gmax.j && g.k < gmax.k;
}

bool Grid3d::isGridIndicesNeighbours(int i1, int j1, int k1, int i2, int j2, int k2) {
    return abs(i1 - i2) <= 1 && abs(j1 - j2) <= 1 && abs(k1 - k2) <= 1;
}

bool Grid3d::isGridIndicesNeighbours(GridIndex g1, int i2, int j2, int k2) {
    return abs(g1.i - i2) <= 1 && abs(g1.j - j2) <= 1 && abs(g1.k - k2) <= 1;
}

bool Grid3d::isGridIndicesNeighbours(int i1, int j1, int k1, GridIndex g2) {
    return abs(i1 - g2.i) <= 1 && abs(j1 - g2.j) <= 1 && abs(k1 - g2.k) <= 1;
}

bool Grid3d::isGridIndicesNeighbours(GridIndex g1, GridIndex g2) {
    return abs(g1.i - g2.i) <= 1 && abs(g1.j - g2.j) <= 1 && abs(g1.k - g2.k) <= 1;
}

bool Grid3d::isGridIndexOnBorder(int i, int j, int k, int imax, int jmax, int kmax) {
    return i == 0 || j == 0 || k == 0 ||
           i == imax - 1 || j == jmax - 1 || k == kmax - 1;
}

bool Grid3d::isGridIndexOnBorder(GridIndex g, int imax, int jmax, int kmax) {
    return g.i == 0 || g.j == 0 || g.k == 0 ||
           g.i == imax - 1 || g.j == jmax - 1 || g.k == kmax - 1;
}

bool Grid3d::isGridIndexOnBorder(int i, int j, int k, GridIndex gmax) {
    return i == 0 || j == 0 || k == 0 ||
           i == gmax.i - 1 || j == gmax.j - 1 || k == gmax.k - 1;
}

bool Grid3d::isGridIndexOnBorder(GridIndex g, GridIndex gmax) {
    return g.i == 0 || g.j == 0 || g.k == 0 ||
           g.i == gmax.i - 1 || g.j == gmax.j - 1 || g.k == gmax.k - 1;
}

void Grid3d::getNeighbourGridIndices6(int i, int j, int k, GridIndex n[6]) {
    n[0] = GridIndex(i-1, j,   k);
    n[1] = GridIndex(i+1, j,   k);
    n[2] = GridIndex(i,   j-1, k);
    n[3] = GridIndex(i,   j+1, k);
    n[4] = GridIndex(i,   j,   k-1);
    n[5] = GridIndex(i,   j,   k+1);
}

void Grid3d::getNeighbourGridIndices6(GridIndex g, GridIndex n[6]) {
    n[0] = GridIndex(g.i-1, g.j,   g.k);
    n[1] = GridIndex(g.i+1, g.j,   g.k);
    n[2] = GridIndex(g.i,   g.j-1, g.k);
    n[3] = GridIndex(g.i,   g.j+1, g.k);
    n[4] = GridIndex(g.i,   g.j,   g.k-1);
    n[5] = GridIndex(g.i,   g.j,   g.k+1);

}

void Grid3d::getNeighbourGridIndices26(int i, int j, int k, GridIndex n[26]) {
    int idx = 0;
    for (int nk = k-1; nk <= k+1; nk++) {
        for (int nj = j-1; nj <= j+1; nj++) {
            for (int ni = i-1; ni <= i+1; ni++) {
                if (!(ni == i && nj == j && nk == k)) {
                    n[idx] = GridIndex(ni, nj, nk);
                    idx++;
                }
            }
        }
    }
}

void Grid3d::getNeighbourGridIndices26(GridIndex g, GridIndex n[26]) {
    int idx = 0;
    for (int nk = g.k-1; nk <= g.k+1; nk++) {
        for (int nj = g.j-1; nj <= g.j+1; nj++) {
            for (int ni = g.i-1; ni <= g.i+1; ni++) {
                if (!(ni == g.i && nj == g.j && nk == g.k)) {
                    n[idx] = GridIndex(ni, nj, nk);
                    idx++;
                }
            }
        }
    }
}

void Grid3d::getNeighbourGridIndices124(int i, int j, int k, GridIndex n[124]) {
    int idx = 0;
    for (int nk = k-2; nk <= k+2; nk++) {
        for (int nj = j-2; nj <= j+2; nj++) {
            for (int ni = i-2; ni <= i+2; ni++) {
                if (!(ni == i && nj == j && nk == k)) {
                    n[idx] = GridIndex(ni, nj, nk);
                    idx++;
                }
            }
        }
    }
}

void Grid3d::getNeighbourGridIndices124(GridIndex g, GridIndex n[124]) {
    int idx = 0;
    for (int nk = g.k-2; nk <= g.k+2; nk++) {
        for (int nj = g.j-2; nj <= g.j+2; nj++) {
            for (int ni = g.i-2; ni <= g.i+2; ni++) {
                if (!(ni == g.i && nj == g.j && nk == g.k)) {
                    n[idx] = GridIndex(ni, nj, nk);
                    idx++;
                }
            }
        }
    }
}

void Grid3d::getGridIndexVertices(int i, int j, int k, GridIndex v[8]) {
    v[0] = GridIndex(i,     j,     k);
    v[1] = GridIndex(i + 1, j,     k);
    v[2] = GridIndex(i + 1, j,     k + 1);
    v[3] = GridIndex(i,     j,     k + 1);
    v[4] = GridIndex(i,     j + 1, k);
    v[5] = GridIndex(i + 1, j + 1, k);
    v[6] = GridIndex(i + 1, j + 1, k + 1);
    v[7] = GridIndex(i,     j + 1, k + 1);
}

void Grid3d::getGridIndexVertices(GridIndex g, GridIndex v[8]) {
    v[0] = GridIndex(g.i, g.j,         g.k);
    v[1] = GridIndex(g.i + 1, g.j,     g.k);
    v[2] = GridIndex(g.i + 1, g.j,     g.k + 1);
    v[3] = GridIndex(g.i,     g.j,     g.k + 1);
    v[4] = GridIndex(g.i,     g.j + 1, g.k);
    v[5] = GridIndex(g.i + 1, g.j + 1, g.k);
    v[6] = GridIndex(g.i + 1, g.j + 1, g.k + 1);
    v[7] = GridIndex(g.i,     g.j + 1, g.k + 1);
}

void Grid3d::getVertexGridIndexNeighbours(int i, int j, int k, GridIndex n[8]) {
    n[0] = GridIndex(i,     j,     k);
    n[1] = GridIndex(i - 1, j,     k);
    n[2] = GridIndex(i,     j,     k - 1);
    n[3] = GridIndex(i - 1, j,     k - 1);
    n[4] = GridIndex(i,     j - 1, k);
    n[5] = GridIndex(i - 1, j - 1, k);
    n[6] = GridIndex(i,     j - 1, k - 1);
    n[7] = GridIndex(i - 1, j - 1, k - 1);
}

void Grid3d::getVertexGridIndexNeighbours(GridIndex v, GridIndex n[8]) {
    n[0] = GridIndex(v.i,     v.j,     v.k);
    n[1] = GridIndex(v.i - 1, v.j,     v.k);
    n[2] = GridIndex(v.i,     v.j,     v.k - 1);
    n[3] = GridIndex(v.i - 1, v.j,     v.k - 1);
    n[4] = GridIndex(v.i,     v.j - 1, v.k);
    n[5] = GridIndex(v.i - 1, v.j - 1, v.k);
    n[6] = GridIndex(v.i,     v.j - 1, v.k - 1);
    n[7] = GridIndex(v.i - 1, v.j - 1, v.k - 1);
}

void Grid3d::getGridIndexBounds(glm::vec3 p, double r, double dx,
                                int imax, int jmax, int kmax, 
                                GridIndex *g1, GridIndex *g2) {
    GridIndex c = positionToGridIndex(p, dx);
    glm::vec3 cpos = GridIndexToPosition(c, dx);
    glm::vec3 trans = p - cpos;
    double inv = 1.0 / dx;

    int gimin = c.i - (int)fmax(0, ceil((r-trans.x)*inv));
    int gjmin = c.j - (int)fmax(0, ceil((r-trans.y)*inv));
    int gkmin = c.k - (int)fmax(0, ceil((r-trans.z)*inv));
    int gimax = c.i + (int)fmax(0, ceil((r-dx+trans.x)*inv));
    int gjmax = c.j + (int)fmax(0, ceil((r-dx+trans.y)*inv));
    int gkmax = c.k + (int)fmax(0, ceil((r-dx+trans.z)*inv));

    *g1 = GridIndex((int)fmax(gimin, 0), 
                      (int)fmax(gjmin, 0), 
                      (int)fmax(gkmin, 0));
    *g2 = GridIndex((int)fmin(gimax, imax-1), 
                      (int)fmin(gjmax, jmax-1), 
                      (int)fmin(gkmax, kmax-1));
}

void Grid3d::getGridIndexBounds(glm::vec3 p, double r, double dx, GridIndex gmax, 
                                   GridIndex *g1, GridIndex *g2) {
    GridIndex c = positionToGridIndex(p, dx);
    glm::vec3 cpos = GridIndexToPosition(c, dx);
    glm::vec3 trans = p - cpos;
    double inv = 1.0 / dx;

    int imin = c.i - (int)fmax(0, ceil((r-trans.x)*inv));
    int jmin = c.j - (int)fmax(0, ceil((r-trans.y)*inv));
    int kmin = c.k - (int)fmax(0, ceil((r-trans.z)*inv));
    int imax = c.i + (int)fmax(0, ceil((r-dx+trans.x)*inv));
    int jmax = c.j + (int)fmax(0, ceil((r-dx+trans.y)*inv));
    int kmax = c.k + (int)fmax(0, ceil((r-dx+trans.z)*inv));

    *g1 = GridIndex((int)fmax(imin, 0), 
                      (int)fmax(jmin, 0), 
                      (int)fmax(kmin, 0));
    *g2 = GridIndex((int)fmin(imax, gmax.i-1), 
                      (int)fmin(jmax, gmax.j-1), 
                      (int)fmin(kmax, gmax.k-1));
}

void Grid3d::getGridIndexBounds(AABB bbox, double dx, 
                                int imax, int jmax, int kmax,
                                GridIndex *g1, GridIndex *g2) {
    glm::vec3 trans = glm::vec3(bbox.width, bbox.height, bbox.depth);
    *g1 = positionToGridIndex(bbox.position, dx);
    *g2 = positionToGridIndex(bbox.position + trans, dx);

    *g1 = GridIndex((int)fmax((*g1).i, 0), 
                    (int)fmax((*g1).j, 0), 
                    (int)fmax((*g1).k, 0));
    *g2 = GridIndex((int)fmin((*g2).i, imax-1), 
                     (int)fmin((*g2).j, jmax-1), 
                     (int)fmin((*g2).k, kmax-1));
}

void Grid3d::getGridIndexBounds(AABB bbox, double dx, GridIndex gmax,
                                GridIndex *g1, GridIndex *g2) {
    glm::vec3 trans = glm::vec3(bbox.width, bbox.height, bbox.depth);
    *g1 = positionToGridIndex(bbox.position, dx);
    *g2 = positionToGridIndex(bbox.position + trans, dx);

    *g1 = GridIndex((int)fmax((*g1).i, 0), 
                    (int)fmax((*g1).j, 0), 
                    (int)fmax((*g1).k, 0));
    *g2 = GridIndex((int)fmin((*g2).i, gmax.i-1), 
                     (int)fmin((*g2).j, gmax.j-1), 
                     (int)fmin((*g2).k, gmax.k-1));
}
