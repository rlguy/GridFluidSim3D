/*
Copyright (c) 2015 Ryan L. Guy

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
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

std::vector<GridIndex> FluidSource::getFluidCells(Array3d<int> &materialGrid,
                                                  double dx) {
    return std::vector<GridIndex>();
}

int FluidSource::getID() {
    return id;
}

void FluidSource::setID(int identifier) {
    id = identifier;
}