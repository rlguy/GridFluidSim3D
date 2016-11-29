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
    closeState();
}

void FluidSimulationSaveState::saveState(std::string filename, FluidSimulation *_fluidsim) {
    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream state(filename.c_str(), std::ios::out | std::ios::binary);

    FLUIDSIM_ASSERT(state.is_open());

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

    // Should a FluidBrickGrid savestate be generated
    bool isFluidBrickGridEnabled = _fluidsim->isBrickOutputEnabled();
    _writeBool(&isFluidBrickGridEnabled, &state);

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

    // chars: diffuse particle types
    _writeBinaryDiffuseParticleTypes(_fluidsim, &state);

    // ints: solid cell indicies in form [i1, j1, k1, i2, j2, k2, ...]
    _writeBinarySolidCellIndices(_fluidsim, &state);

    if (isFluidBrickGridEnabled) {
        _writeBinaryFluidBrickGrid(_fluidsim, &state);
    }

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
                   _readInt(&_numSolidCells, &_loadState) &&
                   _readBool(&_isFluidBrickGridEnabled, &_loadState);

    if (!success) {
        return false;
    }

    _mpPositionOffset = _loadState.tellg();
    _mpVelocityOffset = _mpPositionOffset + _numMarkerParticles*(3*sizeof(float));
    _dpPositionOffset = _mpVelocityOffset + _numMarkerParticles*(3*sizeof(float));
    _dpVelocityOffset = _dpPositionOffset + _numDiffuseParticles*(3*sizeof(float));
    _dpLifetimeOffset = _dpVelocityOffset + _numDiffuseParticles*(3*sizeof(float));
    _dpTypeOffset = _dpLifetimeOffset + _numDiffuseParticles*(sizeof(float));
    _solidCellOffset = _dpTypeOffset + _numDiffuseParticles*sizeof(char);

    if (_isFluidBrickGridEnabled) {
        unsigned int startoffset = _solidCellOffset + _numSolidCells*(3*sizeof(int));

        _loadState.seekg (0, _loadState.end);
        unsigned int endoffset = _loadState.tellg();
        _loadState.seekg (startoffset, _loadState.beg);

        _initializeTempFluidBrickGridFile(startoffset, endoffset);
    }

    _loadState.seekg(0, _loadState.beg);
    _currentOffset = _loadState.tellg();

    _isLoadStateInitialized = true;
    return true;
}

void FluidSimulationSaveState::closeState() {
    if (_loadState.is_open()) {
        _loadState.close();
        _isLoadStateInitialized = false;
    }

    if (_isTempFileInUse) {
        remove(_tempFilename.c_str());
        _isTempFileInUse = false;
    }
}

void FluidSimulationSaveState::getGridDimensions(int *i, int *j, int *k) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    *i = _isize;
    *j = _jsize;
    *k = _ksize;
}

double FluidSimulationSaveState::getCellSize() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _dx;
}

int FluidSimulationSaveState::getCurrentFrame() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _currentFrame;
}

int FluidSimulationSaveState::getNumMarkerParticles() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _numMarkerParticles;
}

int FluidSimulationSaveState::getNumDiffuseParticles() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _numDiffuseParticles;
}

int FluidSimulationSaveState::getNumSolidCells() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _numSolidCells;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticlePositions() {
    return getMarkerParticlePositions(0, _numMarkerParticles);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticleVelocities() {
    return getMarkerParticleVelocities(0, _numMarkerParticles);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticlePositions() {
    return getDiffuseParticlePositions(0, _numDiffuseParticles);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticleVelocities() {
    return getDiffuseParticleVelocities(0, _numDiffuseParticles);
}

std::vector<float> FluidSimulationSaveState::getDiffuseParticleLifetimes() {
    return getDiffuseParticleLifetimes(0, _numDiffuseParticles);
}

std::vector<char> FluidSimulationSaveState::getDiffuseParticleTypes() {
    return getDiffuseParticleTypes(0, _numDiffuseParticles);
}

std::vector<GridIndex> FluidSimulationSaveState::getSolidCells() {
    return getSolidCells(0, _numSolidCells);
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticlePositions(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(endidx >= 0 && endidx <= _numMarkerParticles);

    unsigned int foffset = _mpPositionOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx;
    std::vector<vmath::vec3> positions;
    positions.reserve(n);

    bool success = _readParticleVectors(positions, n, &_loadState);
    FLUIDSIM_ASSERT(success);

    return positions;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getMarkerParticleVelocities(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(endidx >= 0 && endidx <= _numMarkerParticles);

    unsigned int foffset = _mpVelocityOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx;
    std::vector<vmath::vec3> velocities;
    velocities.reserve(n);

    bool success = _readParticleVectors(velocities, n, &_loadState);
    FLUIDSIM_ASSERT(success);

    return velocities;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticlePositions(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(endidx >= 0 && endidx <= _numDiffuseParticles);

    unsigned int foffset = _dpPositionOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx;
    std::vector<vmath::vec3> positions;
    positions.reserve(n);

    bool success = _readParticleVectors(positions, n, &_loadState);
    FLUIDSIM_ASSERT(success);

    return positions;
}

std::vector<vmath::vec3> FluidSimulationSaveState::getDiffuseParticleVelocities(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<vmath::vec3>();
    }

    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(endidx >= 0 && endidx <= _numDiffuseParticles);

    unsigned int foffset = _dpVelocityOffset + startidx*(3*sizeof(float));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx;
    std::vector<vmath::vec3> velocities;
    velocities.reserve(n);

    bool success = _readParticleVectors(velocities, n, &_loadState);
    FLUIDSIM_ASSERT(success);

    return velocities;
}

std::vector<float> FluidSimulationSaveState::getDiffuseParticleLifetimes(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<float>();
    }

    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(startidx <= endidx);
    FLUIDSIM_ASSERT(endidx >= 0 && endidx <= _numDiffuseParticles);

    unsigned int foffset = _dpLifetimeOffset + startidx*sizeof(float);
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx;
    std::vector<float> lifetimes;
    lifetimes.reserve(n);

    bool success = _readParticleLifetimes(lifetimes, n, &_loadState);
    FLUIDSIM_ASSERT(success);

    return lifetimes;
}

std::vector<char> FluidSimulationSaveState::getDiffuseParticleTypes(
                                                        int startidx, int endidx) {

    if (startidx > endidx) {
        return std::vector<char>();
    }

    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(startidx <= endidx);
    FLUIDSIM_ASSERT(endidx >= 0 && endidx <= _numDiffuseParticles);

    unsigned int foffset = _dpTypeOffset + startidx*sizeof(char);
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx;
    std::vector<char> types;
    types.reserve(n);

    bool success = _readParticleTypes(types, n, &_loadState);
    FLUIDSIM_ASSERT(success);

    return types;
}

std::vector<GridIndex> FluidSimulationSaveState::getSolidCells(
                                                        int startidx, int endidx) {
    if (startidx > endidx) {
        return std::vector<GridIndex>();
    }

    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(endidx >= 0 && endidx <= _numSolidCells);

    unsigned int foffset = _solidCellOffset + startidx*(3*sizeof(int));
    _setLoadStateFileOffset(foffset);

    int n = endidx - startidx;
    std::vector<GridIndex> cells;
    cells.reserve(n);

    bool success = _readSolidCells(cells, n, &_loadState);
    FLUIDSIM_ASSERT(success);

    return cells;
}

bool FluidSimulationSaveState::isFluidBrickGridEnabled() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _isFluidBrickGridEnabled;
}

void FluidSimulationSaveState::getFluidBrickGridSaveState(FluidBrickGridSaveState &state) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(_isFluidBrickGridEnabled);
    FLUIDSIM_ASSERT(_isTempFileInUse);
    bool success = state.loadState(_tempFilename);
    FLUIDSIM_ASSERT(success);
    FLUIDSIM_ASSERT(state.isLoadStateInitialized());
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
        int endidx = numWritten + chunksize;
        if (endidx > n) {
            endidx = n;
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
        int endidx = numWritten + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        mvs = _fluidsim->getMarkerParticleVelocities(startidx, endidx);
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
        int endidx = numWritten + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        dps = _fluidsim->getDiffuseParticlePositions(startidx, endidx);
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
        int endidx = numWritten + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        dvs = _fluidsim->getDiffuseParticleVelocities(startidx, endidx);
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
        int endidx = numWritten + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        dfs = _fluidsim->getDiffuseParticleLifetimes(startidx, endidx);
        _writeBinaryVectorf(dfs, state);

        numWritten += chunksize;
    }
}

void FluidSimulationSaveState::_writeBinaryDiffuseParticleTypes(FluidSimulation *_fluidsim,
                                                                std::ofstream *state) {
    int n = _fluidsim->getNumDiffuseParticles();
    int chunksize = _writeChunkSize;
    int numWritten = 0;

    std::vector<char> dts;
    while (numWritten < n) {
        int startidx = numWritten;
        int endidx = numWritten + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        dts = _fluidsim->getDiffuseParticleTypes(startidx, endidx);
        _writeBinaryVectorc(dts, state);

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
        numWritten += (int)indices.size();
    }

}

void FluidSimulationSaveState::_writeBinaryFluidBrickGrid(FluidSimulation *_fluidsim, 
                                                          std::ofstream *state) {
    FLUIDSIM_ASSERT(_fluidsim->isBrickOutputEnabled());
    FluidBrickGrid *brickGrid = _fluidsim->getFluidBrickGrid();

    std::string tempfilename = _getTemporaryFilename();
    FluidBrickGridSaveState brickstate;
    brickstate.saveState(tempfilename, brickGrid);

    _appendFileToSaveState(tempfilename, state);

    int error = remove(tempfilename.c_str());
    FLUIDSIM_ASSERT(error == 0);
}

void FluidSimulationSaveState::_appendFileToSaveState(std::string filename, 
                                                      std::ofstream *state) {
    std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
    *state << file.rdbuf();
    FLUIDSIM_ASSERT(file.good() && state->good());
}

std::string FluidSimulationSaveState::_getTemporaryFilename() {
    int filenamelen = 16;
    std::string filepath = Config::getTempDirectory();
    std::string tfile = filepath + "/" + _getRandomString(filenamelen);

    int numtries = 0;
    while (std::ifstream(tfile)) {
        tfile = _getRandomString(filenamelen);
        numtries++;
        FLUIDSIM_ASSERT(numtries < 100);
    }

    return tfile;
}

std::string FluidSimulationSaveState::_getRandomString(int len) {
    char chars[] = "0123456789abcdefghijklmnopqrstuvwxyz";

    std::string str;
    for (int i = 0; i < len; i++) {
        str += chars[rand() % (sizeof(chars) - 1)];
    }

    return str;
}

void FluidSimulationSaveState::_writeBinaryVector3f(std::vector<vmath::vec3> &vectors, std::ofstream *state) {
    int binsize = 3 * (int)vectors.size() * sizeof(float);
    state->write((char *)&vectors[0], binsize);
}

void FluidSimulationSaveState::_writeBinaryVectorf(std::vector<float> &floats, std::ofstream *state) {
    int binsize = (int)floats.size() * sizeof(float);
    state->write((char *)&floats[0], binsize);
}

void FluidSimulationSaveState::_writeBinaryVectorc(std::vector<char> &chars, std::ofstream *state) {
    int binsize = (int)chars.size() * sizeof(char);
    state->write((char *)&chars[0], binsize);
}

void FluidSimulationSaveState::_writeBinaryVectorGridIndex(std::vector<GridIndex> &indices, 
                                                           std::ofstream *state) {
    int binsize = 3*(int)indices.size()*sizeof(int);
    state->write((char *)&indices[0], binsize);
}

void FluidSimulationSaveState::_writeInt(int *value, std::ofstream *state) {
    state->write((char *)value, sizeof(int));
}

void FluidSimulationSaveState::_writeDouble(double *value, std::ofstream *state) {
    state->write((char *)value, sizeof(double));
}

void FluidSimulationSaveState::_writeBool(bool *value, std::ofstream *state) {
    state->write((char *)value, sizeof(bool));
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

bool FluidSimulationSaveState::_readBool(bool *value, std::ifstream *state) {
    char bin[sizeof(bool)];
    state->read(bin, sizeof(bool));
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(bool));

    return true;
}

bool FluidSimulationSaveState::_initializeTempFluidBrickGridFile(unsigned int startoffset,
                                                                 unsigned int endoffset) {
    _tempFilename = _getTemporaryFilename();
    std::ofstream tempFile(_tempFilename.c_str(), std::ios::out | std::ios::binary);
    _isTempFileInUse = true;

    _setLoadStateFileOffset(startoffset);
    unsigned int filesize = endoffset - startoffset;

    int chunkSize = _writeChunkSize;
    char *bin = new char[chunkSize];
    for (unsigned int i = 0; i < filesize; i += chunkSize) {
        int size = chunkSize;
        if (i + size > filesize) {
            size = filesize - i;
        }

        _loadState.read(bin, size);
        tempFile.write(bin, size);
    }
    delete[] bin;

    _currentOffset = _loadState.tellg();

    return _loadState.good() && tempFile.good();
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

bool FluidSimulationSaveState::_readParticleTypes(std::vector<char> &types, 
                                                  int numParticles,
                                                  std::ifstream *state) {
    int binsize = numParticles*sizeof(char);
    char *bin = new char[binsize];

    state->read(bin, binsize);
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    for (int i = 0; i < numParticles; i++) {
        types.push_back(bin[i]);
    }

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