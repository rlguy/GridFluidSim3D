#include "sphericalfluidsource.h"


SphericalFluidSource::SphericalFluidSource() {
}

SphericalFluidSource::SphericalFluidSource(glm::vec3 pos, double r) : 
                                           FluidSource(pos),
                                           _radius(r > 0.0 ? r : 0.0) {
}

SphericalFluidSource::SphericalFluidSource(glm::vec3 pos, double r, glm::vec3 velocity) : 
                                           FluidSource(pos, velocity),
                                           _radius(r > 0.0 ? r : 0.0) {
}

SphericalFluidSource::~SphericalFluidSource() {
}

void SphericalFluidSource::setRadius(double r) {
    if (r < 0.0) {
        r = 0.0;
    }
    _radius = r;
}

double SphericalFluidSource::getRadius() {
    return _radius;
}

void SphericalFluidSource::setCenter(glm::vec3 p) {
    position = p;
}

void SphericalFluidSource::expand(double val) {
    _radius += val;

    if (_radius < 0.0) {
        _radius = 0.0;
    }
}

std::vector<GridIndex> SphericalFluidSource::getNewFluidCells(Array3d<int> &materialGrid,
                                                              double dx) {
    if (!isActive) {
        return std::vector<GridIndex>();
    }

    if (sourceType == T_OUTFLOW) {
        return std::vector<GridIndex>();
    }

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

std::vector<GridIndex> SphericalFluidSource::getFluidCells(Array3d<int> &materialGrid,
                                                           double dx) {
    if (!isActive) {
        return std::vector<GridIndex>();
    }

    if (sourceType == T_INFLOW) {
        return std::vector<GridIndex>();
    }

    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    std::vector<GridIndex> overlappingIndices;
    std::vector<GridIndex> fluidCells;

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (int i = 0; i < overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid(g) == M_FLUID) {
            fluidCells.push_back(g);
        }
    }

    return fluidCells;
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
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
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
