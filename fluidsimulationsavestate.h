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
#pragma once

#include <vector>
#include <fstream>

#include "macvelocityfield.h"
#include "array3d.h"
#include "glm/glm.hpp"

class FluidSimulation;

class FluidSimulationSaveState
{
public:
    FluidSimulationSaveState();
    ~FluidSimulationSaveState();

    void saveState(std::string filename, FluidSimulation *fluidsim);
    bool loadState(std::string filename);

    void getGridDimensions(int *i, int *j, int *);
    double getCellSize();
    int getCurrentFrame();
    std::vector<glm::vec3> getMarkerParticlePositions();
    std::vector<glm::vec3> getMarkerParticleVelocities();
    std::vector<GridIndex> getSolidCellIndices();
    bool isLoadStateInitialized();

private:

    struct LoadStateData {
        int i;
        int j;
        int k;
        double dx;
        int currentFrame;

        std::vector<glm::vec3> markerParticlePositions;
        std::vector<glm::vec3> markerParticleVelocities;
        std::vector<GridIndex> solidCellIndices;

        LoadStateData() {}
    };

    void _writeInt(int *value, std::ofstream *state);
    void _writeDouble(double *value, std::ofstream *state);
    void _writeBinaryMarkerParticlePositions(FluidSimulation *_fluidsim,
                                             std::ofstream *state);
    void _writeBinaryMarkerParticleVelocities(FluidSimulation *_fluidsim,
                                              std::ofstream *state);
    void _writeSolidCellIndices(FluidSimulation *sim, std::ofstream *state);
    int _getNumSolidCells(FluidSimulation *sim);

    bool _readInt(int *value, std::ifstream *state);
    bool _readDouble(double *value, std::ifstream *state);
    bool _readMarkerParticlePositions(std::vector<glm::vec3> &particles, 
                                      int numParticles,
                                      std::ifstream *state);
    bool _readMarkerParticleVelocities(std::vector<glm::vec3> &velocities, 
                                       int numParticles,
                                       std::ifstream *state);
    bool _readSolidCellIndices(std::vector<GridIndex> &indices, 
                               int numIndices,
                               std::ifstream *state);

    bool _isLoadStateInitialized = false;
    int _width, _height, _depth;
    int M_SOLID = 2;

    LoadStateData _stateData;

};

