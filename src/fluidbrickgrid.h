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
#ifndef FLUIDBRICKGRID_H
#define FLUIDBRICKGRID_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include "array3d.h"
#include "aabb.h"
#include "levelset.h"
#include "fluidmaterialgrid.h"
#include "trianglemesh.h"
#include "vmath.h"
#include "gridindexvector.h"
#include "fluidbrickgridsavestate.h"
#include "brick.h"
#include "fluidsimassert.h"

class FluidBrickGrid
{
public:
    FluidBrickGrid();
    FluidBrickGrid(int isize, int jsize, int ksize, double dx, AABB brick);
    FluidBrickGrid(FluidBrickGridSaveState &state);
    ~FluidBrickGrid();

    void getGridDimensions(int *i, int *j, int *k);
    double getCellSize();
    void getBrickGridDimensions(int *i, int *j, int *k);
    AABB getBrickAABB();
    void setBrickDimensions(double width, double height, double depth);
    void setBrickDimensions(AABB brick);
    bool getBrickMesh(TriangleMesh &mesh);
    bool isBrickMeshReady();
    void update(LevelSet &levelset, 
                FluidMaterialGrid &materialGrid,
                std::vector<vmath::vec3> &markerParticles, 
                double dt);
    void saveState(std::string filename);
    int getBrickGridQueueSize();
    int getNumUpdates();
    void getDensityGridCurrentDensityValues(Array3d<float> &grid);
    void getDensityGridTargetDensityValues(Array3d<float> &grid);
    void getDensityGridVelocityValues(Array3d<float> &grid);
    Array3d<Brick>* getPointerToBrickGridQueue();
    bool isInitialized();

private:

    struct DensityNode {
        float currentDensity = 0.0f;
        float targetDensity = 0.0f;
        float intensityVelocity = 0.0f;
    };

    void _initialize();
    void _initializeFromSaveState(FluidBrickGridSaveState &state);
    void _initializeDensityGridFromSaveState(FluidBrickGridSaveState &state);
    void _initializeBrickGridFromSaveState(FluidBrickGridSaveState &state);
    void _initializeBrickGridQueueFromSaveState(FluidBrickGridSaveState &state);
    void _initializeBrickGrid();
    void _reset();
    void _updateDensityGrid(std::vector<vmath::vec3> &particles, double dt);
    void _updateTargetDensities(std::vector<vmath::vec3> &particles);
    void _updateDensities(double dt);
    void _updateDensity(int i, int j, int k, double dt);
    void _updateBrickGrid(LevelSet &levelset,
                          FluidMaterialGrid &materialGrid);
    float _getBrickIntensity(int i, int j, int k);
    bool _isBrickNextToActiveNeighbour(int i, int j, int k);
    void _postProcessBrickGrid();
    void _removeStrayBricks();
    void _removeSmallBrickStructures();
    void _mergeBrickGrids();
    void _getNewBrickLocations(Array3d<Brick> &b1, Array3d<Brick> &b2,
                               Array3d<bool> &newBricks);
    void _getbrickStructures(Array3d<Brick> &brickGrid, Array3d<bool> &newBricks, 
                         std::vector<GridIndexVector> &brickStructures);
    void _getConnectedBricks(int i, int j, int k, Array3d<Brick> &brickGrid,
                                                  Array3d<bool> &newBricks, 
                                                  GridIndexVector &connectedBricks);
    void _removeInvalidbrickStructures(Array3d<Brick> &brickCurrent,
                                   Array3d<Brick> &brickNext,
                                   std::vector<GridIndexVector> &brickStructures);
    bool _isBrickMassInBrickGrid(GridIndexVector &cells, Array3d<Brick> &brickGrid);
    void _removeBrickStructureFromBrickGrid(GridIndexVector &cells, Array3d<Brick> &brickGrid);

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    AABB _brick;
    Array3d<DensityNode> _densityGrid;
    Array3d<Brick> _brickGrid;
    Array3d<Brick> _brickGridQueue[3];
    int _brickGridQueueSize = 0;
    Array3d<Brick> _currentBrickGrid;
    bool _isCurrentBrickGridReady = false;

    int _minParticleDensityCount = 0;
    int _maxParticleDensityCount = 8;
    float _maxIntensityVelocity = 10.0f;
    float _maxIntensityAcceleration = 10.0f;
    float _decelerationRadius = 0.05f;
    unsigned int _minNumberOfBricksInStructure = 0;

    int _numUpdates = 0;
    bool _isInitialized = false;
};

#endif
