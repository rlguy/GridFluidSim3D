/*
Copyright (c) 2016 Ryan L. Guy

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
    if (_loadState.is_open()) {
        _loadState.close();
    }
}

void FluidSimulationSaveState::saveState(std::string filename, FluidSimulation *_fluidsim) {

    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream state(filename.c_str(), std::ios::out | std::ios::binary);

    assert(state.is_open());

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

    // next frame to be processed
    int currentFrame = _fluidsim->getCurrentFrame();
    _writeInt(&currentFrame, &state);

    // number of marker particles
    int n = _fluidsim->getNumMarkerParticles();
    _writeInt(&n, &state);

    // number of diffuse particles
    n = _fluidsim->getNumDiffuseParticles();
    _writeInt(&n, &state);

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
    _writeBinarySolidCellIndices(_fluidsim, &state);

    state.close();
}

bool FluidSimulationSaveState::loadState(std::string filename) {

    _loadState.open(filename.c_str(), std::ios::in | std::ios::binary);
    
    if (!_loadState.is_open()) {
        return false;
    }

    bool success = _readInt(&_isize, &_loadState) &&
                   _readInt(&_jsize, &_loadState) &&
                   _readInt(&_ksize, &_loadState) &&
                   _readDouble(&_dx, &_loadState) &&
                   _readInt(&_currentFrame, &_loadState) &&
                   _readInt(&_numMarkerParticles, &_loadState) &&
                   _readInt(&_numDiffuseParticles, &_loadState) &&
                   _readInt(&_numSolidCells, &_loadState);

    if (!success) {
        return false;
    }

    _mpPositionOffset = _loadState.tellg();
    _mpVelocityOffset = _mpPositionOffset + _numMarkerParticles*(3*sizeof(float));
    _dpPositionOffset = _mpVelocityOffset + _numMarkerParticles*(3*sizeof(float));
    _dpVelocityOffset = _dpPositionOffset + _numDiffuseParticles*(3*sizeof(float));
    _dpLifetimeOffset = _dpVelocityOffset + _numDiffuseParticles*(3*sizeof(float));
    _solidCellOffset = _dpLifetimeOffset + _numDiffuseParticles*sizeof(float);
    unsigned int endoff = _solidCellOffset + _numSolidCells*(3*sizeof(int));

    _loadState.seekg (0, _loadState.end);
    _oefOffset = _loadState.tellg();
    _loadState.seekg (0, _loadState.beg);

    if (endoff != _oefOffset) {
        return false;
    }

    _currentOffset = _loadState.tellg();

    _isLoadStateInitialized = true;
    return true;
}

void FluidSimulationSaveState::closeState() {
    if (_loadState.is_open()) {
        _loadState.close();
        _isLoadStateInitialized = false;
    }
}

void FluidSimulationSaveState::getGridDimensions(int *i, int *j, int *k) {
    assert(_isLoadStateInitialized);
    *i = _isize;
    *j = _jsize;
    *k = _ksize;
}

double FluidSimulationSaveState::getCellSize() {
    assert(_isLoadStateInitialized);
    return _dx;
}

int FluidSimulationSaveState::getCurrentFrame() {
    assert(_isLoadStateInitialized);
    return _currentFrame;
}

int FluidSimulationSaveState::getNumMarkerParticles() {
    assert(_isLoadStateInitialized);
    return _numMarkerParticles;
}

int FluidSimulationSaveState::getNumDiffuseParticles() {
    assert(_isLoadStateInitialized);
    return _numDiffuseParticles;
}

int FluidSimulationSaveState::getNumSolidCells() {
    assert(_isLoadStateInitialized);
    return _numSolidCells;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticlePositions() {
    return getMarkerParticlePositions(0, _numMarkerParticles - 1);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticleVelocities() {
    return getMarkerParticleVelocities(0, _numMarkerParticles - 1);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticlePositions() {
    return getDiffuseParticlePositions(0, _numDiffuseParticles - 1);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticleVelocities() {
    return getDiffuseParticleVelocities(0, _numDiffuseParticles - 1);
}

std::vector<float> FluidSimulationSaveState::getDiffuseParticleLifetimes() {
    return getDiffuseParticleLifetimes(0, _numDiffuseParticles - 1);
}

std::vector<GridIndex> FluidSimulationSaveState::getSolidCells() {
    return getSolidCells(0, _numSolidCells - 1);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticlePositions(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    assert(_isLoadStateInitialized);
    assert(endidx >= 0 && endidx < _numMarkerParticles);

    unsigned int foffset = _mpPositionOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx + 1;
    std::vector<vmath::vec3> positions;
    positions.reserve(n);

    _readParticleVectors(positions, n, &_loadState);

    return positions;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticleVelocities(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    assert(_isLoadStateInitialized);
    assert(endidx >= 0 && endidx < _numMarkerParticles);

    unsigned int foffset = _mpVelocityOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx + 1;
    std::vector<vmath::vec3> velocities;
    velocities.reserve(n);

    _readParticleVectors(velocities, n, &_loadState);

    return velocities;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticlePositions(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    assert(_isLoadStateInitialized);
    assert(endidx >= 0 && endidx < _numDiffuseParticles);

    unsigned int foffset = _dpPositionOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx + 1;
    std::vector<vmath::vec3> positions;
    positions.reserve(n);

    _readParticleVectors(positions, n, &_loadState);

    return positions;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticleVelocities(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    assert(_isLoadStateInitialized);
    assert(endidx >= 0 && endidx < _numDiffuseParticles);

    unsigned int foffset = _dpVelocityOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx + 1;
    std::vector<vmath::vec3> velocities;
    velocities.reserve(n);

    _readParticleVectors(velocities, n, &_loadState);

    return velocities;
}

std::vector<float> FluidSimulationSaveState::getDiffuseParticleLifetimes(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<float>();
    }

    assert(_isLoadStateInitialized);
    assert(startidx <= endidx);
    assert(endidx >= 0 && endidx < _numDiffuseParticles);

    unsigned int foffset = _dpLifetimeOffset + startidx*sizeof(float);
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx + 1;
    std::vector<float> lifetimes;
    lifetimes.reserve(n);

    _readParticleLifetimes(lifetimes, n, &_loadState);

    return lifetimes;
}

std::vector<GridIndex> FluidSimulationSaveState::getSolidCells(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<GridIndex>();
    }

    assert(_isLoadStateInitialized);
    assert(endidx >= 0 && endidx < _numSolidCells);

    unsigned int foffset = _solidCellOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx + 1;
    std::vector<GridIndex> cells;
    cells.reserve(n);

    _readSolidCells(cells, n, &_loadState);

    return cells;
}

bool FluidSimulationSaveState::isLoadStateInitialized() {
    return _isLoadStateInitialized;
}

void FluidSimulationSaveState::_writeBinaryMarkerParticlePositions(FluidSimulation *_fluidsim,
                                                                   std::ofstream *state) {
    int n = _fluidsim->getNumMarkerParticles();
    int chunksize = _writeChunkSize;
    int numWritten = 0;

    std::vector<vmath::vec3> mps;
    while (numWritten < n) {
        int startidx = numWritten;
        int endidx = numWritten + chunksize - 1;
        if (endidx >= n) {
            endidx = n - 1;
        }

        mps = _fluidsim->getMarkerParticlePositions(startidx, endidx);
        _writeBinaryVector3f(mps, state);

        numWritten += chunksize;
    }
}

void FluidSimulationSaveState::_writeBinaryMarkerParticleVelocities(FluidSimulation *_fluidsim,
                                                                    std::ofstream *state) {
    int n = _fluidsim->getNumMarkerParticles();
    int chunksize = _writeChunkSize;
    int numWritten = 0;

    std::vector<vmath::vec3> mvs;
    while (numWritten < n) {
        int startidx = numWritten;
        int endidx = numWritten + chunksize - 1;
        if (endidx >= n) {
            endidx = n - 1;
        }

        std::vector<vmath::vec3> mvs = _fluidsim->getMarkerParticleVelocities(startidx, endidx);
        _writeBinaryVector3f(mvs, state);

        numWritten += chunksize;
    }
}

void FluidSimulationSaveState::_writeBinaryDiffuseParticlePositions(FluidSimulation *_fluidsim,
                                                                    std::ofstream *state) {
    int n = _fluidsim->getNumDiffuseParticles();
    int chunksize = _writeChunkSize;
    int numWritten = 0;

    std::vector<vmath::vec3> dps;
    while (numWritten < n) {
        int startidx = numWritten;
        int endidx = numWritten + chunksize - 1;
        if (endidx >= n) {
            endidx = n - 1;
        }

        std::vector<vmath::vec3> dps = _fluidsim->getDiffuseParticlePositions(startidx, endidx);
        _writeBinaryVector3f(dps, state);

        numWritten += chunksize;
    }
}

void FluidSimulationSaveState::_writeBinaryDiffuseParticleVelocities(FluidSimulation *_fluidsim,
                                                                     std::ofstream *state) {
    int n = _fluidsim->getNumDiffuseParticles();
    int chunksize = _writeChunkSize;
    int numWritten = 0;

    std::vector<vmath::vec3> dvs;
    while (numWritten < n) {
        int startidx = numWritten;
        int endidx = numWritten + chunksize - 1;
        if (endidx >= n) {
            endidx = n - 1;
        }

        std::vector<vmath::vec3> dvs = _fluidsim->getDiffuseParticleVelocities(startidx, endidx);
        _writeBinaryVector3f(dvs, state);

        numWritten += chunksize;
    }
}

void FluidSimulationSaveState::_writeBinaryDiffuseParticleLifetimes(FluidSimulation *_fluidsim,
                                                                    std::ofstream *state) {
    int n = _fluidsim->getNumDiffuseParticles();
    int chunksize = _writeChunkSize;
    int numWritten = 0;

    std::vector<float> dfs;
    while (numWritten < n) {
        int startidx = numWritten;
        int endidx = numWritten + chunksize - 1;
        if (endidx >= n) {
            endidx = n - 1;
        }

        std::vector<float> dfs = _fluidsim->getDiffuseParticleLifetimes(startidx, endidx);
        _writeBinaryVectorf(dfs, state);

        numWritten += chunksize;
    }
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

void FluidSimulationSaveState::_writeBinarySolidCellIndices(FluidSimulation *_fluidsim, 
                                                            std::ofstream *state) {

    int n = _getNumSolidCells(_fluidsim);
    int chunksize = _writeChunkSize;
    int numWritten = 0;

    std::vector<GridIndex> indices;
    indices.reserve(chunksize);
    GridIndex startIndex(0, 0, 0);
    while (numWritten < n) {
        indices.clear();

        for (int k = startIndex.k; k < _depth; k++) {
            for (int j = startIndex.j; j < _height; j++) {
                for (int i = startIndex.i; i < _width; i++) {

                    if ((int)indices.size() == chunksize) {
                        startIndex = GridIndex(i, j, k);
                        goto endLoop;
                    }

                    if (_fluidsim->getMaterial(i, j, k) == Material::solid) {
                        indices.push_back(GridIndex(i, j, k));
                    }

                }
                startIndex.i = 0;
            }
            startIndex.j = 0;
        }
        endLoop:

        _writeBinaryVectorGridIndex(indices, state);
        numWritten += indices.size();
    }

}

void FluidSimulationSaveState::_writeBinaryVector3f(std::vector<vmath::vec3> &vectors, std::ofstream *state) {
    int binsize = 3 * vectors.size() * sizeof(float);
    state->write((char *)&vectors[0], binsize);
}

void FluidSimulationSaveState::_writeBinaryVectorf(std::vector<float> &floats, std::ofstream *state) {
    int binsize = floats.size() * sizeof(float);
    state->write((char *)&floats[0], binsize);
}

void FluidSimulationSaveState::_writeBinaryVectorGridIndex(std::vector<GridIndex> &indices, 
                                                           std::ofstream *state) {
    int binsize = 3*indices.size()*sizeof(int);
    state->write((char *)&indices[0], binsize);
}

void FluidSimulationSaveState::_writeInt(int *value, std::ofstream *state) {
    state->write((char *)value, sizeof(int));
}

void FluidSimulationSaveState::_writeDouble(double *value, std::ofstream *state) {
    state->write((char *)value, sizeof(double));
}

void FluidSimulationSaveState::_setLoadStateFileOffset(unsigned int foffset) {
    if (foffset != _currentOffset) {
        _loadState.seekg(foffset);
        _currentOffset = _loadState.tellg();
    }
}

bool FluidSimulationSaveState::_readInt(int *value, std::ifstream *state) {
    char bin[sizeof(int)];
    state->read(bin, sizeof(int));
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(int));

    return true;
}

bool FluidSimulationSaveState::_readDouble(double *value, std::ifstream *state) {
    char bin[sizeof(double)];
    state->read(bin, sizeof(double));
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(double));

    return true;
}

bool FluidSimulationSaveState::_readParticleVectors(std::vector<vmath::vec3> &vectors, 
                                                    int numParticles,
                                                    std::ifstream *state) {
    int binsize = 3*numParticles*sizeof(float);
    char *bin = new char[binsize];

    state->read(bin, binsize);
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    vmath::vec3 *positions = (vmath::vec3 *)bin;
    vectors.assign(positions, positions + numParticles);
    delete[] bin;

    return true;
}

bool FluidSimulationSaveState::_readParticleLifetimes(std::vector<float> &pls, 
                                                      int numParticles,
                                                      std::ifstream *state) {
    int binsize = numParticles*sizeof(float);
    char *bin = new char[binsize];

    state->read(bin, binsize);
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    float *lifetimes = (float *)bin;
    pls.assign(lifetimes, lifetimes + numParticles);
    delete[] bin;

    return true;
}

bool FluidSimulationSaveState::_readSolidCells(std::vector<GridIndex> &indices, 
                                               int numIndices,
                                               std::ifstream *state) {

    int binsize = 3*numIndices*sizeof(int);
    char *bin = new char[binsize];

    state->read(bin, binsize);
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    GridIndex *data = (GridIndex *)bin;
    indices.assign(data, data + numIndices);
    delete[] bin;

    return true;
}