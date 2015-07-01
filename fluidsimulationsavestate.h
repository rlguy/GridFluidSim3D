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
    std::vector<glm::vec3> getMarkerParticles();
    void getVelocityField(Array3d<double> &U, 
                          Array3d<double> &V, 
                          Array3d<double> &W);
    std::vector<GridIndex> getSolidCellIndices();
    bool isLoadStateInitialized();

private:
    struct MACVelocity {
        Array3d<double> U, V, W;
        int width, height, depth;

        MACVelocity() {};
        MACVelocity(int i, int j, int k) : U(i + 1, j, k),
                                           V(i, j + 1, k),
                                           W(i, j, k + 1),
                                           width(i), height(j), depth(k) {};
    };

    struct LoadStateData {
        int i;
        int j;
        int k;
        double dx;
        int currentFrame;

        std::vector<glm::vec3> markerParticles;
        MACVelocity vgrid;
        std::vector<GridIndex> solidCellIndices;

        LoadStateData() {}
    };

    void _writeInt(int *value, std::ofstream *state);
    void _writeDouble(double *value, std::ofstream *state);
    void _writeBinaryMarkerParticlePositions(std::vector<glm::vec3> &mps,
                                             std::ofstream *state);
    void _writeBinaryVelocitiesU(MACVelocityField *vgrid, std::ofstream *state);
    void _writeBinaryVelocitiesV(MACVelocityField *vgrid, std::ofstream *state);
    void _writeBinaryVelocitiesW(MACVelocityField *vgrid, std::ofstream *state);
    void _writeSolidCellIndices(FluidSimulation *sim, std::ofstream *state);
    int _getNumSolidCells(FluidSimulation *sim);

    bool _readInt(int *value, std::ifstream *state);
    bool _readDouble(double *value, std::ifstream *state);
    bool _readMarkerParticles(std::vector<glm::vec3> &particles, 
                              int numParticles,
                              std::ifstream *state);
    bool _readMACVelocityGrid(MACVelocity &vgrid, std::ifstream *state);
    bool _readSolidCellIndices(std::vector<GridIndex> &indices, 
                               int numIndices,
                               std::ifstream *state);

    bool _isLoadStateInitialized = false;
    int _width, _height, _depth;
    int M_SOLID = 2;

    LoadStateData _stateData;

};

