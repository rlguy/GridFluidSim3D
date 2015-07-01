#include "sphericalfluidsource.h"


SphericalFluidSource::SphericalFluidSource() {
}

SphericalFluidSource::SphericalFluidSource(glm::vec3 pos, double r) : 
                                           FluidSource(pos),
                                           _radius(r) {
}

SphericalFluidSource::SphericalFluidSource(glm::vec3 pos, double r, glm::vec3 velocity) : 
                                           FluidSource(pos, velocity),
                                           _radius(r) {
}

SphericalFluidSource::~SphericalFluidSource() {
}

void SphericalFluidSource::setRadius(double r) {
    _radius = r;
}

double SphericalFluidSource::getRadius() {
    return _radius;
}

std::vector<GridIndex> SphericalFluidSource::getNewFluidCells(Array3d<int> &materialGrid,
                                                              double dx) {
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    std::vector<GridIndex> overlappingIndices;
    std::vector<GridIndex> newFluidCells;

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid(g) == M_AIR) {
            newFluidCells.push_back(g);
        }
    }

    return newFluidCells;
}

void SphericalFluidSource::_getOverlappingGridIndices(std::vector<GridIndex> &indices,
                                                      int isize, int jsize, int ksize, 
                                                      double dx) {

    double r = _radius;
    AABB bbox = AABB(position - glm::vec3(r, r, r), 2*r, 2*r, 2*r);
    GridIndex gmin, gmax;
    getGridIndexBounds(bbox, isize, jsize, ksize, dx, &gmin, &gmax);

    double rsq = r*r;
    double distsq;
    glm::vec3 p;
    glm::vec3 v;
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                p = GridIndexToCellCenter(i, j, k, dx);
                v = p - position;
                distsq = glm::dot(v, v);
                if (distsq < rsq) {
                    indices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

}
