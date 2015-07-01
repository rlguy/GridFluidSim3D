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
    _bbox = AABB(_bbox.position, w, _bbox.height, _bbox.depth);
}

void CuboidFluidSource::setHeight(double h) {
    _bbox = AABB(_bbox.position, _bbox.width, h, _bbox.depth);
}

void CuboidFluidSource::setDepth(double d) {
    _bbox = AABB(_bbox.position, _bbox.width, _bbox.height, d);
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
}

AABB CuboidFluidSource::getBoundingBox() {
    return _bbox;
}

void CuboidFluidSource::setCenter(glm::vec3 pos) {
    glm::vec3 c = getCenter();
    translate(pos - c);
}

glm::vec3 CuboidFluidSource::getCenter() {
    return glm::vec3(_bbox.position.x + 0.5*_bbox.width, 
                     _bbox.position.y + 0.5*_bbox.width, 
                     _bbox.position.z + 0.5*_bbox.width);
}

void CuboidFluidSource::expand(double value) {
    glm::vec3 p = _bbox.position - glm::vec3(value, value, value);
    _bbox = AABB(p, _bbox.width + 2*value, 
                    _bbox.height + 2*value, 
                    _bbox.depth + 2*value);
}

std::vector<GridIndex> CuboidFluidSource::getNewFluidCells(Array3d<int> &materialGrid,
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

void CuboidFluidSource::_getOverlappingGridIndices(std::vector<GridIndex> &indices,
                                                   int isize, int jsize, int ksize, 
                                                   double dx) {
    GridIndex gmin, gmax;
    getGridIndexBounds(_bbox, isize, jsize, ksize, dx, &gmin, &gmax);

    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                indices.push_back(GridIndex(i, j, k));
            }
        }
    }
}