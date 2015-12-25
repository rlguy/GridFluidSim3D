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
#include "fluidsimulationsavestate.h"
#include "fluidsimulation.h"


FluidSimulationSaveState::FluidSimulationSaveState() {
}


FluidSimulationSaveState::~FluidSimulationSaveState() {
}

void FluidSimulationSaveState::saveState(std::string filename, FluidSimulation *_fluidsim) {

    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream state(filename.c_str(), std::ios::out | std::ios::binary);

    int i, j, k;
    _fluidsim->getGridDimensions(&i, &j, &k);
    double dx = _fluidsim->getCellSize();
    _width = i;
    _height = j;
    _depth = k;

    // i, j, k, dx
    _writeInt(&i, &state);
    _writeInt(&j, &state);
    _writeInt(&k, &state);
    _writeDouble(&dx, &state);

    // number of marker particles
    int n = _fluidsim->getNumMarkerParticles();
    _writeInt(&n, &state);

    // number of diffuse particles
    n = _fluidsim->getNumDiffuseParticles();
    _writeInt(&n, &state);

    // next frame to be processed
    int currentFrame = _fluidsim->getCurrentFrame();
    _writeInt(&currentFrame, &state);

    // number of solid cell indices
    int numIndices = _getNumSolidCells(_fluidsim);
    _writeInt(&numIndices, &state);

    // floats: marker particle positions in form [x1, y1, z1, x2, y2, z2, ...]
    _writeBinaryMarkerParticlePositions(_fluidsim, &state);

    // floats: marker particle velocities in form [x1, y1, z1, x2, y2, z2, ...]
    _writeBinaryMarkerParticleVelocities(_fluidsim, &state);

    // floats: diffuse particle positions in form [x1, y1, z1, x2, y2, z2, ...]
    _writeBinaryDiffuseParticlePositions(_fluidsim, &state);

    // floats: diffuse particle velocities in form [x1, y1, z1, x2, y2, z2, ...]
    _writeBinaryDiffuseParticleVelocities(_fluidsim, &state);

    // floats: diffuse particle lifetimes
    _writeBinaryDiffuseParticleLifetimes(_fluidsim, &state);

    // ints: solid cell indicies in form [i1, j1, k1, i2, j2, k2, ...]
    _writeSolidCellIndices(_fluidsim, &state);

    state.close();
}

bool FluidSimulationSaveState::loadState(std::string filename) {
    _stateData = LoadStateData();

    std::ifstream state(filename.c_str(), std::ios::in | std::ios::binary);

    int numMarkerParticles, numDiffuseParticles, numSolidCellIndices;

    bool success = _readInt(&_stateData.i, &state) &&
                   _readInt(&_stateData.j, &state) &&
                   _readInt(&_stateData.k, &state) &&
                   _readDouble(&_stateData.dx, &state) &&
                   _readInt(&numMarkerParticles, &state) &&
                   _readInt(&numDiffuseParticles, &state) &&
                   _readInt(&_stateData.currentFrame, &state) &&
                   _readInt(&numSolidCellIndices, &state);

    if (!success) {
        return false;
    }

    success = _readParticlePositions(_stateData.markerParticlePositions, 
                                     numMarkerParticles, &state);
    if (!success) {
        return false;
    }

    success = _readParticleVelocities(_stateData.markerParticleVelocities, 
                                      numMarkerParticles, &state);
    if (!success) {
        return false;
    }

    success = _readParticlePositions(_stateData.diffuseParticlePositions, 
                                     numDiffuseParticles, &state);
    if (!success) {
        return false;
    }

    success = _readParticleVelocities(_stateData.diffuseParticleVelocities, 
                                      numDiffuseParticles, &state);
    if (!success) {
        return false;
    }

    success = _readParticleLifetimes(_stateData.diffuseParticleLifetimes, 
                                     numDiffuseParticles, &state);
    if (!success) {
        return false;
    }

    success = _readSolidCellIndices(_stateData.solidCellIndices, numSolidCellIndices, &state);
    if (!success) {
        return false;
    }

    _isLoadStateInitialized = true;
    return true;
}

void FluidSimulationSaveState::getGridDimensions(int *i, int *j, int *k) {
    assert(_isLoadStateInitialized);
    *i = _stateData.i;
    *j = _stateData.j;
    *k = _stateData.k;
}

double FluidSimulationSaveState::getCellSize() {
    assert(_isLoadStateInitialized);
    return _stateData.dx;
}

int FluidSimulationSaveState::getCurrentFrame() {
    assert(_isLoadStateInitialized);
    return _stateData.currentFrame;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticlePositions() {
    assert(_isLoadStateInitialized);
    return _stateData.markerParticlePositions;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticleVelocities() {
    assert(_isLoadStateInitialized);
    return _stateData.markerParticleVelocities;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticlePositions() {
    assert(_isLoadStateInitialized);
    return _stateData.diffuseParticlePositions;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticleVelocities() {
    assert(_isLoadStateInitialized);
    return _stateData.diffuseParticleVelocities;
}

std::vector<float> FluidSimulationSaveState::getDiffuseParticleLifetimes() {
    assert(_isLoadStateInitialized);
    return _stateData.diffuseParticleLifetimes;
}

std::vector<GridIndex> FluidSimulationSaveState::getSolidCellIndices() {
    assert(_isLoadStateInitialized);
    return _stateData.solidCellIndices;
}

bool FluidSimulationSaveState::isLoadStateInitialized() {
    return _isLoadStateInitialized;
}

void FluidSimulationSaveState::_writeBinaryMarkerParticlePositions(FluidSimulation *_fluidsim,
                                                                   std::ofstream *state) {
    std::vector<vmath::vec3> mps = _fluidsim->getMarkerParticlePositions();
    _writeBinaryVector3f(mps, state);
}

void FluidSimulationSaveState::_writeBinaryMarkerParticleVelocities(FluidSimulation *_fluidsim,
                                                                    std::ofstream *state) {
    std::vector<vmath::vec3> mvs = _fluidsim->getMarkerParticleVelocities();
    _writeBinaryVector3f(mvs, state);
}

void FluidSimulationSaveState::_writeBinaryDiffuseParticlePositions(FluidSimulation *_fluidsim,
                                                                    std::ofstream *state) {
    std::vector<vmath::vec3> dps = _fluidsim->getDiffuseParticlePositions();
    _writeBinaryVector3f(dps, state);
}

void FluidSimulationSaveState::_writeBinaryDiffuseParticleVelocities(FluidSimulation *_fluidsim,
                                                                     std::ofstream *state) {
    std::vector<vmath::vec3> dvs = _fluidsim->getDiffuseParticleVelocities();
    _writeBinaryVector3f(dvs, state);
}

void FluidSimulationSaveState::_writeBinaryDiffuseParticleLifetimes(FluidSimulation *_fluidsim,
                                                                    std::ofstream *state) {
    std::vector<float> dfs = _fluidsim->getDiffuseParticleLifetimes();
    _writeBinaryVectorf(dfs, state);
}

void FluidSimulationSaveState::_writeBinaryVector3f(std::vector<vmath::vec3> &vectors, std::ofstream *state) {
    int binsize = 3 * vectors.size() * sizeof(float);
    char *storage = new char[binsize];

    int fsize = (int)sizeof(float);
    int offset = 0;
    char position[3*sizeof(float)];

    vmath::vec3 v;
    for (unsigned int i = 0; i < vectors.size(); i++) {
        v = vectors[i];

        memcpy(position, &v.x, fsize);
        memcpy(position + fsize, &v.y, fsize);
        memcpy(position + 2*fsize, &v.z, fsize);
        memcpy(storage + offset, position, 3*fsize);

        offset += 3*fsize;
    }

    state->write(storage, binsize);
    delete[] storage;
}

void FluidSimulationSaveState::_writeBinaryVectorf(std::vector<float> &floats, std::ofstream *state) {
    int binsize = floats.size() * sizeof(float);
    char *storage = new char[binsize];

    int fsize = (int)sizeof(float);
    int offset = 0;

    float f;
    for (unsigned int i = 0; i < floats.size(); i++) {
        f = floats[i];
        memcpy(storage + offset, &f, fsize);
        offset += fsize;
    }

    state->write(storage, binsize);
    delete[] storage;
}

void FluidSimulationSaveState::_writeInt(int *value, std::ofstream *state) {
    char bin[sizeof(int)];
    memcpy(bin, value, sizeof(int));
    state->write(bin, sizeof(int));
}

void FluidSimulationSaveState::_writeDouble(double *value, std::ofstream *state) {
    char bin[sizeof(double)];
    memcpy(bin, value, sizeof(double));
    state->write(bin, sizeof(double));
}

int FluidSimulationSaveState::_getNumSolidCells(FluidSimulation *sim) {
    int count = 0;
    for (int k = 0; k < _depth; k++) {
        for (int j = 0; j < _height; j++) {
            for (int i = 0; i < _width; i++) {
                if (sim->getMaterial(i, j, k) == Material::solid) {
                    count++;
                }
            }
        }
    }

    return count;
}

void FluidSimulationSaveState::_writeSolidCellIndices(FluidSimulation *sim, 
                                                      std::ofstream *state) {
    std::vector<GridIndex> indices;
    for (int k = 0; k < _depth; k++) {
        for (int j = 0; j < _height; j++) {
            for (int i = 0; i < _width; i++) {
                if (sim->getMaterial(i, j, k) == Material::solid) {
                    indices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

    int *data = new int[3*indices.size()];
    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        data[3*i] = g.i;
        data[3*i + 1] = g.j;
        data[3*i + 2] = g.k;
    }

    int binsize = 3*indices.size()*sizeof(int);
    char *storage = new char[binsize];
    memcpy(storage, data, binsize);
    delete[] data;
    state->write(storage, binsize);

    delete[] storage;
}

bool FluidSimulationSaveState::_readInt(int *value, std::ifstream *state) {
    char bin[sizeof(int)];
    state->read(bin, sizeof(int));

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(int));

    return true;
}

bool FluidSimulationSaveState::_readDouble(double *value, std::ifstream *state) {
    char bin[sizeof(double)];
    state->read(bin, sizeof(double));

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(double));

    return true;
}

bool FluidSimulationSaveState::_readParticlePositions(std::vector<vmath::vec3> &particles, 
                                                            int numParticles,
                                                            std::ifstream *state) {
    int binsize = 3*numParticles*sizeof(float);
    char *bin = new char[binsize];

    state->read(bin, binsize);

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    float *positions = new float[3*numParticles];
    memcpy(positions, bin, binsize);
    delete[] bin;
    for (int i = 0; i < 3*numParticles; i += 3) {
        float x = positions[i];
        float y = positions[i + 1];
        float z = positions[i + 2];

        particles.push_back(vmath::vec3(x, y, z));
    }
    delete[] positions;

    return true;
}

bool FluidSimulationSaveState::_readParticleVelocities(std::vector<vmath::vec3> &pvs, 
                                                             int numParticles,
                                                             std::ifstream *state) {
    int binsize = 3*numParticles*sizeof(float);
    char *bin = new char[binsize];

    state->read(bin, binsize);

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    float *velocities = new float[3*numParticles];
    memcpy(velocities, bin, binsize);
    delete[] bin;
    for (int i = 0; i < 3*numParticles; i += 3) {
        float x = velocities[i];
        float y = velocities[i + 1];
        float z = velocities[i + 2];

        pvs.push_back(vmath::vec3(x, y, z));
    }
    delete[] velocities;

    return true;
}

bool FluidSimulationSaveState::_readParticleLifetimes(std::vector<float> &pls, 
                                                      int numParticles,
                                                      std::ifstream *state) {
    int binsize = numParticles*sizeof(float);
    char *bin = new char[binsize];

    state->read(bin, binsize);

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    float *lifetimes = new float[numParticles];
    memcpy(lifetimes, bin, binsize);
    delete[] bin;
    for (int i = 0; i < numParticles; i++) {
        pls.push_back(lifetimes[i]);
    }
    delete[] lifetimes;

    return true;
}

bool FluidSimulationSaveState::_readSolidCellIndices(std::vector<GridIndex> &indices, 
                                                     int numIndices,
                                                     std::ifstream *state) {

    int binsize = 3*numIndices*sizeof(int);
    char *bin = new char[binsize];

    state->read(bin, binsize);

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    int *data = new int[3*numIndices];
    memcpy(data, bin, binsize);
    delete[] bin;
    for (int idx = 0; idx < 3*numIndices; idx += 3) {
        int i = data[idx];
        int j = data[idx + 1];
        int k = data[idx + 2];

        indices.push_back(GridIndex(i, j, k));
    }
    delete[] data;

    return true;
}