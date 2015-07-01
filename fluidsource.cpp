#include "fluidsource.h"


FluidSource::FluidSource() : position(0, 0, 0),
                             velocity(0.0, 0.0, 0.0),
                             direction(1.0, 0.0, 0.0) {
}

FluidSource::FluidSource(glm::vec3 pos) : position(pos),
                                          velocity(0.0, 0.0, 0.0),
                                          direction(1.0, 0.0, 0.0) {
}

FluidSource::FluidSource(glm::vec3 pos, glm::vec3 vel) : 
                                          position(pos),
                                          velocity(vel),
                                          direction(glm::normalize(velocity)) {
    if (!(glm::length(velocity) > 0.0)) {
        direction = glm::vec3(1.0, 0.0, 0.0);
    }
}


FluidSource::~FluidSource() {
}

void FluidSource::setPosition(glm::vec3 pos) {
    position = pos;
}

glm::vec3 FluidSource::getPosition() {
   return position;
}

void FluidSource::translate(glm::vec3 trans) {
    position += trans;
}

void FluidSource::setVelocity(glm::vec3 v) {
    velocity = v;
    if (glm::length(v) > 0.0) {
        direction = glm::normalize(v);
    } else {
        direction = glm::vec3(0.0, 0.0, 0.0);
    }
}

void FluidSource::setDirection(glm::vec3 dir) {
    float length = glm::length(dir);
    if (!(length > 0.0f)) {
        return;
    }

    direction = glm::normalize(dir);
    velocity = glm::length(velocity) * velocity;
}

glm::vec3 FluidSource::getVelocity() {
    return velocity;
}

void FluidSource::setAsInFlow() {
    sourceType = T_INFLOW;
}

void FluidSource::setAsOutFlow() {
    sourceType = T_OUTFLOW;
}

int FluidSource::getSourceType() {
    return sourceType;
}

void FluidSource::activate() {
    isActive = true;
}

void FluidSource::deactivate() {
    isActive = false;
}

std::vector<GridIndex> FluidSource::getNewFluidCells(Array3d<int> &materialGrid,
                                        double dx) {
    return std::vector<GridIndex>();
}

void FluidSource::getGridIndexBounds(AABB bbox, int i, int j, int k, double dx, 
                                     GridIndex *gmin, GridIndex *gmax) {
    glm::vec3 trans = glm::vec3(bbox.width, bbox.height, bbox.depth);
    *gmin = positionToGridIndex(bbox.position, dx);
    *gmax = positionToGridIndex(bbox.position + trans, dx);

    *gmin = GridIndex(fmax((*gmin).i, 0), fmax((*gmin).j, 0), fmax((*gmin).k, 0));
    *gmax = GridIndex(fmin((*gmax).i, i-1), fmin((*gmax).j, j-1), fmin((*gmax).k, k-1));
}