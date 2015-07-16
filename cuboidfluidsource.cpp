#include "cuboidfluidsource.h"


CuboidFluidSource::CuboidFluidSource() {
}

CuboidFluidSource::CuboidFluidSource(glm::vec3 position, 
                                     double w, double h, double d) : 
                                     FluidSource(position), 
                                     _bbox(position, w, h, d) {
}

CuboidFluidSource::CuboidFluidSource(AABB bbox) : 
                                     FluidSource(bbox.position), 
                                     _bbox(bbox) {
}

CuboidFluidSource::CuboidFluidSource(glm::vec3 position, 
                                     double w, double h, double d,
                                     glm::vec3 velocity) : 
                                     FluidSource(position, velocity), 
                                     _bbox(position, w, h, d) {
}

CuboidFluidSource::CuboidFluidSource(AABB bbox, glm::vec3 velocity) : 
                                     FluidSource(bbox.position, velocity), 
                                     _bbox(bbox) {
}

CuboidFluidSource::~CuboidFluidSource() {
}

void CuboidFluidSource::setWidth(double w) {
    _bbox = AABB(position, w, _bbox.height, _bbox.depth);
}

void CuboidFluidSource::setHeight(double h) {
    _bbox = AABB(position, _bbox.width, h, _bbox.depth);
}

void CuboidFluidSource::setDepth(double d) {
    _bbox = AABB(position, _bbox.width, _bbox.height, d);
}

double CuboidFluidSource::getWidth() {
    return _bbox.width;
}

double CuboidFluidSource::getHeight() {
    return _bbox.height;
}

double CuboidFluidSource::getDepth() {
    return _bbox.depth;
}

void CuboidFluidSource::setBoundingBox(AABB bbox) {
    _bbox = bbox;
    position = bbox.position;
}

AABB CuboidFluidSource::getBoundingBox() {
    _bbox.position = position;
    return _bbox;
}

void CuboidFluidSource::setCenter(glm::vec3 pos) {
    glm::vec3 c = getCenter();
    translate(pos - c);
    _bbox.position = pos;
}

glm::vec3 CuboidFluidSource::getCenter() {
    return glm::vec3(position.x + 0.5*_bbox.width, 
                     position.y + 0.5*_bbox.width, 
                     position.z + 0.5*_bbox.width);
}

void CuboidFluidSource::expand(double value) {
    glm::vec3 p = position - glm::vec3(value, value, value);
    _bbox = AABB(p, _bbox.width + 2*value, 
                    _bbox.height + 2*value, 
                    _bbox.depth + 2*value);
}

std::vector<GridIndex> CuboidFluidSource::getNewFluidCells(Array3d<unsigned char> &materialGrid,
                                                              double dx) {
    if (!isActive) {
        return std::vector<GridIndex>();
    }

    if (sourceType == T_OUTFLOW) {
        return std::vector<GridIndex>();
    }

    _bbox.position = position;
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    std::vector<GridIndex> overlappingIndices;
    std::vector<GridIndex> newFluidCells;

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (int i = 0; i < (int)overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid(g) == M_AIR) {
            newFluidCells.push_back(g);
        }
    }

    return newFluidCells;
}

std::vector<GridIndex> CuboidFluidSource::getFluidCells(Array3d<unsigned char> &materialGrid,
                                                        double dx) {
    if (!isActive) {
        return std::vector<GridIndex>();
    }

    if (sourceType == T_INFLOW) {
        return std::vector<GridIndex>();
    }

     _bbox.position = position;
    int w = materialGrid.width;
    int h = materialGrid.height;
    int d = materialGrid.depth;
    std::vector<GridIndex> overlappingIndices;
    std::vector<GridIndex> fluidCells;

    _getOverlappingGridIndices(overlappingIndices, w, h, d, dx);

    GridIndex g;
    for (int i = 0; i < (int)overlappingIndices.size(); i++) {
        g = overlappingIndices[i];
        if (materialGrid(g) == M_FLUID) {
            fluidCells.push_back(g);
        }
    }

    return fluidCells;
}

void CuboidFluidSource::_getOverlappingGridIndices(std::vector<GridIndex> &indices,
                                                   int isize, int jsize, int ksize, 
                                                   double dx) {
    GridIndex gmin, gmax;
    getGridIndexBounds(_bbox, isize, jsize, ksize, dx, &gmin, &gmax);

    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                indices.push_back(GridIndex(i, j, k));
            }
        }
    }
}