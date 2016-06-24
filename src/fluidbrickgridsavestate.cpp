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
#include "fluidbrickgridsavestate.h"
#include "fluidbrickgrid.h"

FluidBrickGridSaveState::FluidBrickGridSaveState() {
}


FluidBrickGridSaveState::~FluidBrickGridSaveState() {
    closeState();
}

void FluidBrickGridSaveState::saveState(std::string filename, FluidBrickGrid *brickgrid) {

    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream state(filename.c_str(), std::ios::out | std::ios::binary);

    FLUIDSIM_ASSERT(state.is_open());

    int i, j, k;
    brickgrid->getGridDimensions(&i, &j, &k);

    _isize = i;
    _jsize = j;
    _ksize = k;
    _dx = brickgrid->getCellSize();
    _brickAABB = brickgrid->getBrickAABB();
    _brickGridQueueSize = brickgrid->getBrickGridQueueSize();
    _numUpdates = brickgrid->getNumUpdates();

    // i, j, k, dx
    _writeInt(&_isize, &state);
    _writeInt(&_jsize, &state);
    _writeInt(&_ksize, &state);
    _writeDouble(&_dx, &state);

    // brick width, height, depth
    _writeDouble(&_brickAABB.width, &state);
    _writeDouble(&_brickAABB.height, &state);
    _writeDouble(&_brickAABB.depth, &state);

    // queuesize, num updates
    _writeInt(&_brickGridQueueSize, &state);
    _writeInt(&_numUpdates, &state);

    // in form: 
    // {{list current densities (floats)}, {list target densities (floats)},
    // {list intensity velocities (floats)}}
    _writeBinaryDensityGrid(brickgrid, &state);

    // in form:
    // {{isActive  values for queue[0] (bools)}, 
    //  {intensity values for queue[0] (floats)}, 
    //  {isActive  values for queue[1] (bools)}, 
    //  {intensity values for queue[1] (floats)}, 
    //  {isActive  values for queue[2] (bools)},
    //  {intensity values for queue[2] (floats)}}
    _writeBinaryBrickGridQueue(brickgrid, &state);

    state.close();
}

bool FluidBrickGridSaveState::loadState(std::string filename) {

    _loadState.open(filename.c_str(), std::ios::in | std::ios::binary);
    
    if (!_loadState.is_open()) {
        return false;
    }

    bool success = _readInt(&_isize, &_loadState) &&
                   _readInt(&_jsize, &_loadState) &&
                   _readInt(&_ksize, &_loadState) &&
                   _readDouble(&_dx, &_loadState) &&
                   _readDouble(&(_brickAABB.width), &_loadState) &&
                   _readDouble(&(_brickAABB.height), &_loadState) &&
                   _readDouble(&(_brickAABB.depth), &_loadState) &&
                   _readInt(&_brickGridQueueSize, &_loadState) &&
                   _readInt(&_numUpdates, &_loadState);

    if (!success) {
        return false;
    }

    int numGridElements = _isize * _jsize * _ksize;

    int bisize, bjsize, bksize;
    _getBrickGridDimensions(&bisize, &bjsize, &bksize);
    int numBrickGridElements = bisize*bjsize*bksize;

    _currentDensityOffset = _loadState.tellg();
    _targetDensityOffset = _currentDensityOffset + numGridElements * sizeof(float);
    _velocityDensityOffset = _targetDensityOffset + numGridElements * sizeof(float);

    unsigned int endoff = _velocityDensityOffset + numGridElements * sizeof(float);
    if (_brickGridQueueSize >= 1) {
        _brickGridOffset1 = _velocityDensityOffset + numGridElements * sizeof(float);
        endoff = _brickGridOffset1 + numBrickGridElements * (sizeof(bool) + sizeof(float));
    }

    if (_brickGridQueueSize >= 2) {
        _brickGridOffset2 = _brickGridOffset1 + numBrickGridElements * (sizeof(bool) + sizeof(float));
        endoff = _brickGridOffset2 + numBrickGridElements * (sizeof(bool) + sizeof(float));
    }

    if (_brickGridQueueSize >= 3) {
        _brickGridOffset3 = _brickGridOffset2 + numBrickGridElements * (sizeof(bool) + sizeof(float));
        endoff = _brickGridOffset3 + numBrickGridElements * (sizeof(bool) + sizeof(float));
    }

    _loadState.seekg (0, _loadState.end);
    _eofOffset = _loadState.tellg();
    _loadState.seekg (0, _loadState.beg);

    if (endoff != _eofOffset) {
        return false;
    }

    _currentOffset = _loadState.tellg();

    _isLoadStateInitialized = true;

    return true;
}

bool FluidBrickGridSaveState::isLoadStateInitialized() {
    return _isLoadStateInitialized;
}

void FluidBrickGridSaveState::closeState() {
    if (_loadState.is_open()) {
        _loadState.close();
        _isLoadStateInitialized = false;
    }
}

void FluidBrickGridSaveState::getGridDimensions(int *i, int *j, int *k) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    *i = _isize;
    *j = _jsize;
    *k = _ksize;
}

void FluidBrickGridSaveState::getBrickGridDimensions(int *bi, int *bj, int *bk) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    _getBrickGridDimensions(bi, bj, bk);
}

double FluidBrickGridSaveState::getCellSize() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _dx;
}

AABB FluidBrickGridSaveState::getBrickAABB() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _brickAABB;
}

int FluidBrickGridSaveState::getBrickGridQueueSize() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _brickGridQueueSize;
}

int FluidBrickGridSaveState::getNumUpdates() {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    return _numUpdates;
}


void FluidBrickGridSaveState::getCurrentDensityGrid(Array3d<float> &grid) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(grid.width == _isize &&
           grid.height == _jsize &&
           grid.depth == _ksize);

    int binsize = _isize * _jsize * _ksize * sizeof(float);
    _setLoadStateFileOffset(_currentDensityOffset);

    char *bin = (char *)grid.getRawArray();
    FLUIDSIM_ASSERT(_readLoadState(bin, binsize));
}

void FluidBrickGridSaveState::getTargetDensityGrid(Array3d<float> &grid) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(grid.width == _isize &&
           grid.height == _jsize &&
           grid.depth == _ksize);

    int binsize = _isize * _jsize * _ksize * sizeof(float);
    _setLoadStateFileOffset(_targetDensityOffset);

    char *bin = (char *)grid.getRawArray();
    FLUIDSIM_ASSERT(_readLoadState(bin, binsize));
}

void FluidBrickGridSaveState::getVelocityDensityGrid(Array3d<float> &grid) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);
    FLUIDSIM_ASSERT(grid.width == _isize &&
           grid.height == _jsize &&
           grid.depth == _ksize);

    int binsize = _isize * _jsize * _ksize * sizeof(float);
    _setLoadStateFileOffset(_velocityDensityOffset);

    char *bin = (char *)grid.getRawArray();
    FLUIDSIM_ASSERT(_readLoadState(bin, binsize));
}

void FluidBrickGridSaveState::getBrickActivityGrid(Array3d<bool> &grid, int idx) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);

    int bi, bj, bk;
    _getBrickGridDimensions(&bi, &bj, &bk);
    FLUIDSIM_ASSERT(grid.width == bi &&
           grid.height == bj &&
           grid.depth == bk);

    FLUIDSIM_ASSERT(idx >= 0 && idx <= 2);
    FLUIDSIM_ASSERT(idx < _brickGridQueueSize);

    if (idx == 0) {
        _setLoadStateFileOffset(_brickGridOffset1);
    } else if (idx == 1) {
        _setLoadStateFileOffset(_brickGridOffset2);
    } else if (idx == 2) {
        _setLoadStateFileOffset(_brickGridOffset3);
    }

    int binsize = bi * bj * bk * sizeof(bool);
    char *bin = (char *)grid.getRawArray();
    FLUIDSIM_ASSERT(_readLoadState(bin, binsize));
}

void FluidBrickGridSaveState::getBrickIntensityGrid(Array3d<float> &grid, int idx) {
    FLUIDSIM_ASSERT(_isLoadStateInitialized);

    int bi, bj, bk;
    _getBrickGridDimensions(&bi, &bj, &bk);
    FLUIDSIM_ASSERT(grid.width == bi &&
           grid.height == bj &&
           grid.depth == bk);

    FLUIDSIM_ASSERT(idx >= 0 && idx <= 2);
    FLUIDSIM_ASSERT(idx < _brickGridQueueSize);

    int dataOffset = bi * bj * bk * sizeof(bool);

    if (idx == 0) {
        _setLoadStateFileOffset(_brickGridOffset1 + dataOffset);
    } else if (idx == 1) {
        _setLoadStateFileOffset(_brickGridOffset2 + dataOffset);
    } else if (idx == 2) {
        _setLoadStateFileOffset(_brickGridOffset3 + dataOffset);
    }

    int binsize = bi * bj * bk * sizeof(float);
    char *bin = (char *)grid.getRawArray();
    FLUIDSIM_ASSERT(_readLoadState(bin, binsize));
}

void FluidBrickGridSaveState::_writeInt(int *value, std::ofstream *state) {
    state->write((char *)value, sizeof(int));
}

void FluidBrickGridSaveState::_writeDouble(double *value, std::ofstream *state) {
    state->write((char *)value, sizeof(double));
}

void FluidBrickGridSaveState::_writeBinaryDensityGrid(FluidBrickGrid *brickgrid, 
                                                      std::ofstream *state) {
    Array3d<float> tempgrid(_isize, _jsize, _ksize);
    brickgrid->getDensityGridCurrentDensityValues(tempgrid);
    _writeBinaryArray3df(tempgrid, state);

    brickgrid->getDensityGridTargetDensityValues(tempgrid);
    _writeBinaryArray3df(tempgrid, state);

    brickgrid->getDensityGridVelocityValues(tempgrid);
    _writeBinaryArray3df(tempgrid, state);
}

void FluidBrickGridSaveState::_writeBinaryBrickGridQueue(FluidBrickGrid *brickgrid, 
                                                         std::ofstream *state) {
    Array3d<Brick> *queue = brickgrid->getPointerToBrickGridQueue();
    
    Array3d<Brick> *b1 = &(queue[0]);
    Array3d<Brick> *b2 = &(queue[1]);
    Array3d<Brick> *b3 = &(queue[2]);

    if (b1->getNumElements() > 0) {
        _writeBinaryBrickGrid(b1, state);
    }
    if (b2->getNumElements() > 0) {
        _writeBinaryBrickGrid(b2, state);
    }
    if (b3->getNumElements() > 0) {
        _writeBinaryBrickGrid(b3, state);
    }
}

void FluidBrickGridSaveState::_writeBinaryBrickGrid(Array3d<Brick> *grid,
                                                    std::ofstream *state) {
    Array3d<bool> isActiveGrid(grid->width, grid->height, grid->depth);
    Array3d<float> intensityGrid(grid->width, grid->height, grid->depth);

    for (int k = 0; k < grid->depth; k++) {
        for (int j = 0; j < grid->height; j++) {
            for (int i = 0; i < grid->width; i++) {
                isActiveGrid.set(i, j, k, grid->get(i, j, k).isActive);
                intensityGrid.set(i, j, k, grid->get(i, j, k).intensity);
            }
        }
    }
    _writeBinaryArray3db(isActiveGrid, state);
    _writeBinaryArray3df(intensityGrid, state);
}

void FluidBrickGridSaveState::_writeBinaryArray3df(Array3d<float> &grid, 
                                                   std::ofstream *state) {
    int binsize = grid.width * grid.height * grid.depth * sizeof(float);
    char *data = (char *)grid.getRawArray();
    state->write(data, binsize);
}

void FluidBrickGridSaveState::_writeBinaryArray3db(Array3d<bool> &grid, 
                                                   std::ofstream *state) {
    int binsize = grid.width * grid.height * grid.depth * sizeof(bool);
    char *data = (char *)grid.getRawArray();
    state->write(data, binsize);
}

void FluidBrickGridSaveState::_setLoadStateFileOffset(unsigned int foffset) {
    if (foffset != _currentOffset) {
        _loadState.seekg(foffset);
        _currentOffset = _loadState.tellg();
    }
}

bool FluidBrickGridSaveState::_readInt(int *value, std::ifstream *state) {
    char bin[sizeof(int)];
    state->read(bin, sizeof(int));
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(int));

    return true;
}

bool FluidBrickGridSaveState::_readDouble(double *value, std::ifstream *state) {
    char bin[sizeof(double)];
    state->read(bin, sizeof(double));
    _currentOffset = _loadState.tellg();

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(double));

    return true;
}

void FluidBrickGridSaveState::_getBrickGridDimensions(int *bi, int *bj, int *bk) {
    double width = _isize*_dx;
    double height = _jsize*_dx;
    double depth = _ksize*_dx;
    *bi = (int)ceil(width / _brickAABB.width);
    *bj = (int)ceil(height / _brickAABB.height);
    *bk = (int)ceil(depth / _brickAABB.depth);
}

bool FluidBrickGridSaveState::_readLoadState(char *dest, unsigned int numBytes) {
    _loadState.read(dest, numBytes);
    _currentOffset = _loadState.tellg();
    return _loadState.good();
}

