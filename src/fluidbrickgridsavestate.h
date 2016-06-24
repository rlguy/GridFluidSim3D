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
#ifndef FLUIDBRICKGRIDSAVESTATE_H
#define FLUIDBRICKGRIDSAVESTATE_H

#include <vector>
#include <fstream>

#include "array3d.h"
#include "vmath.h"
#include "aabb.h"
#include "brick.h"
#include "fluidsimassert.h"

class FluidBrickGrid;

class FluidBrickGridSaveState
{
public:
    FluidBrickGridSaveState();
    ~FluidBrickGridSaveState();

    void saveState(std::string filename, FluidBrickGrid *brickgrid);
    bool loadState(std::string filename);
    void closeState();

    bool isLoadStateInitialized();
    void getGridDimensions(int *i, int *j, int *k);
    void getBrickGridDimensions(int *bi, int *bj, int *bk);
    double getCellSize();
    AABB getBrickAABB();
    int getBrickGridQueueSize();
    int getNumUpdates();
    void getCurrentDensityGrid(Array3d<float> &grid);
    void getTargetDensityGrid(Array3d<float> &grid);
    void getVelocityDensityGrid(Array3d<float> &grid);
    void getBrickActivityGrid(Array3d<bool> &grid, int idx);
    void getBrickIntensityGrid(Array3d<float> &grid, int idx);

private:

    void _writeInt(int *value, std::ofstream *state);
    void _writeDouble(double *value, std::ofstream *state);
    void _writeBinaryDensityGrid(FluidBrickGrid *brickgrid, 
                                 std::ofstream *state);
    void _writeBinaryBrickGridQueue(FluidBrickGrid *brickgrid, 
                                    std::ofstream *state);
    void _writeBinaryBrickGrid(Array3d<Brick> *grid,
                               std::ofstream *state);
    void _writeBinaryArray3df(Array3d<float> &grid, 
                              std::ofstream *state);
    void _writeBinaryArray3db(Array3d<bool> &grid, 
                              std::ofstream *state);

    void _setLoadStateFileOffset(unsigned int foffset);
    bool _readInt(int *value, std::ifstream *state);
    bool _readDouble(double *value, std::ifstream *state);
    void _getBrickGridDimensions(int *bi, int *bj, int *bk);
    bool _readLoadState(char *dest, unsigned int numBytes);

    std::ifstream _loadState;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;
    AABB _brickAABB;
    int _brickGridQueueSize = 0;
    int _numUpdates = 0;

    unsigned int _currentDensityOffset = 0;
    unsigned int _targetDensityOffset = 0;
    unsigned int _velocityDensityOffset = 0;
    unsigned int _brickGridOffset1 = 0;
    unsigned int _brickGridOffset2 = 0;
    unsigned int _brickGridOffset3 = 0;
    unsigned int _eofOffset = 0;
    unsigned int _currentOffset = 0;

    bool _isLoadStateInitialized = false;
};

#endif
