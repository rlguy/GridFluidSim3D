#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>

#include <gl\glew.h>
#include <SDL_opengl.h>
#include <gl\glu.h>
#include <assert.h>

#include "stopwatch.h"
#include "MACVelocityField.h"
#include "array3d.h"
#include "implicitfield.h"
#include "glm/glm.hpp"

class FluidSimulation
{
public:
    FluidSimulation();
    FluidSimulation(int x_voxels, int y_voxels, int z_voxels, double cell_size);
    ~FluidSimulation();

    void update(double dt);
    void draw();

    void run();
    void pause();
        
    double getCellSize() { return dx; }
    void getGridDimensions(int *i, int *j, int *k) { *i = i_voxels; *j = j_voxels; *k = k_voxels; }
    void getSimulationDimensions(double *w, double *h, double *d) { *w = (double)i_voxels*dx;
                                                                    *h = (double)j_voxels*dx;
                                                                    *d = (double)k_voxels*dx; }
    double getSimulationWidth() {  return (double)i_voxels*dx; }
    double getSimulationHeight() { return (double)j_voxels*dx; }
    double getSimulationDepth() {  return (double)k_voxels*dx; }

    int getMaterial(int i, int j, int k) { return materialGrid(i, j, k); }

    std::vector<ImplicitPointData> getImplicitFluidPoints();
    std::vector<glm::vec3> getMarkerParticles();
    Array3d<int> getLayerGrid() { return layerGrid; }

    void addBodyForce(double fx, double fy, double fz) { addBodyForce(glm::vec3(fx, fy, fz)); }
    void addBodyForce(glm::vec3 f);
    void setBodyForce(double fx, double fy, double fz) { setBodyForce(glm::vec3(fx, fy, fz)); }
    void setBodyForce(glm::vec3 f);

    double getDensity() { return density; }
    double setDensity(double p) { assert(p > 0); density = p; }

    void addImplicitFluidPoint(double x, double y, double z, double r) {
        addImplicitFluidPoint(glm::vec3(x, y, z), r);
    }
    void addImplicitFluidPoint(glm::vec3 p, double radius);

    void gridIndexToPosition(int i, int j, int k, double *x, double *y, double *z);
    void gridIndexToCellCenter(int i, int j, int k, double *x, double *y, double *z);
    void positionToGridIndex(double x, double y, double z, int *i, int *j, int *k);

private:
    struct MarkerParticle {
        glm::vec3 position = glm::vec3(0.0, 0.0, 0.0);
        int i = 0;
        int j = 0;
        int k = 0;

        MarkerParticle() : position(glm::vec3(0.0, 0.0, 0.0)), i(0), j(0), k(0) {}

        MarkerParticle(glm::vec3 p, int ii, int jj, int kk) : position(p),
                                                              i(ii), j(jj), k(kk) {}

        MarkerParticle(double x, double y, double z, int ii, int jj, int kk) : 
                        position(glm::vec3(x, y, z)),
                        i(ii), j(jj), k(kk) {}
    };

    struct MatrixCoefficients {
        Array3d<double> diag;
        Array3d<double> plusi;
        Array3d<double> plusj;
        Array3d<double> plusk;
        int width, height, depth;

        MatrixCoefficients() : width(0), height(0), depth(0) {} 
        MatrixCoefficients(int i, int j, int k) : diag(Array3d<double>(i, j, k, 0.0)),
                                                  plusi(Array3d<double>(i, j, k, 0.0)),
                                                  plusj(Array3d<double>(i, j, k, 0.0)),
                                                  plusk(Array3d<double>(i, j, k, 0.0)),
                                                  width(i), height(j), depth(k) {};
    };

    int M_AIR = 0;
    int M_FLUID = 1;
    int M_SOLID = 2;

    void _initializeSimulation();
    void _initializeSolidCells();
    void _initializeFluidMaterial();
    void _addMarkerParticlesToCell(int i, int j, int k);

    double _calculateNextTimeStep();
    void _stepFluid(double dt);
    void _updateFluidCells();
    void _extrapolateFluidVelocities();
    void _resetExtrapolatedFluidVelocities();
    int _updateExtrapolationLayers();
    void _updateExtrapolationLayer(int layerIndex);
    void _getNeighbourGridIndices6(int i, int j, int k, GridIndex n[6]);
    void _getNeighbourGridIndices26(int i, int j, int k, GridIndex n[26]);
    void _extrapolateVelocitiesForLayerIndex(int layerIndex);
    double _getExtrapolatedVelocityForFaceU(int i, int j, int k, int layerIndex);
    double _getExtrapolatedVelocityForFaceV(int i, int j, int k, int layerIndex);
    double _getExtrapolatedVelocityForFaceW(int i, int j, int k, int layerIndex);
    glm::vec3 _getExtrapolatedVelocityAtPosition(glm::vec3 p);
    void _applyBodyForcesToVelocityField(double dt);
    void _advectVelocityField(double dt);
    void _backwardsAdvectVelocity(glm::vec3 p0, glm::vec3 v0, double dt, glm::vec3 *p1, glm::vec3 *v1);
    bool _integrateVelocity(glm::vec3 p0, glm::vec3 v0, double dt, glm::vec3 *p1, glm::vec3 *v1);
    void _updatePressureGrid(double dt);
    void _calculateMatrixCoefficients(MatrixCoefficients &A, double dt);
    void _advanceMarkerParticles(double dt);


    glm::vec3 _RK2(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK3(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK4(glm::vec3 p0, glm::vec3 v0, double dt);

    inline bool _isCellAir(int i, int j, int k) { return materialGrid(i, j, k) == M_AIR; }
    inline bool _isCellFluid(int i, int j, int k) { return materialGrid(i, j, k) == M_FLUID; }
    inline bool _isCellSolid(int i, int j, int k) { return materialGrid(i, j, k) == M_SOLID; }

    inline bool _isFaceBorderingFluidU(int i, int j, int k) {
        if (i == i_voxels) {
            return materialGrid(i - 1, j, k) == M_FLUID;
        } else if (i > 0) {
            return materialGrid(i, j, k) == M_FLUID || materialGrid(i-1, j, k) == M_FLUID;
        } else {
            return materialGrid(i, j, k) == M_FLUID;
        }
    }

    inline bool _isFaceBorderingFluidV(int i, int j, int k) {
        if (j == j_voxels) {
            return materialGrid(i, j - 1, k) == M_FLUID;
        } else if (j > 0) {
            return materialGrid(i, j, k) == M_FLUID || materialGrid(i, j - 1, k) == M_FLUID;
        } else {
            return materialGrid(i, j, k) == M_FLUID;
        }
    }

    inline bool _isFaceBorderingFluidW(int i, int j, int k) {
        if (k == k_voxels) {
            return materialGrid(i, j, k - 1) == M_FLUID;
        }
        else if (k > 0) {
            return materialGrid(i, j, k) == M_FLUID || materialGrid(i, j, k - 1) == M_FLUID;
        }
        else {
            return materialGrid(i, j, k) == M_FLUID;
        }
    }

    inline bool _isFaceBorderingLayerIndexU(int i, int j, int k, int layer) {
        if (i == i_voxels) {
            return layerGrid(i - 1, j, k) == layer;
        }
        else if (i > 0) {
            return layerGrid(i, j, k) == layer || layerGrid(i - 1, j, k) == layer;
        }
        else {
            return layerGrid(i, j, k) == layer;
        }
    }

    inline bool _isFaceBorderingLayerIndexV(int i, int j, int k, int layer) {
        if (j == j_voxels) {
            return layerGrid(i, j - 1, k) == layer;
        }
        else if (j > 0) {
            return layerGrid(i, j, k) == layer || layerGrid(i, j - 1, k) == layer;
        }
        else {
            return layerGrid(i, j, k) == layer;
        }
    }

    inline bool _isFaceBorderingLayerIndexW(int i, int j, int k, int layer) {
        if (k == k_voxels) {
            return layerGrid(i, j, k - 1) == layer;
        }
        else if (k > 0) {
            return layerGrid(i, j, k) == layer || layerGrid(i, j, k - 1) == layer;
        }
        else {
            return layerGrid(i, j, k) == layer;
        }
    }

    inline bool _isFaceVelocityExtrapolatedU(int i, int j, int k) {
        if (i == i_voxels) {
            return layerGrid(i - 1, j, k) >= 1.0;
        }
        else if (i > 0) {
            return layerGrid(i, j, k) >= 1.0 || layerGrid(i - 1, j, k) >= 1.0;
        }
        else {
            return layerGrid(i, j, k) >= 1.0;
        }
    }

    inline bool _isFaceVelocityExtrapolatedV(int i, int j, int k) {
        if (j == j_voxels) {
            return layerGrid(i, j - 1, k) >= 1.0;
        }
        else if (j > 0) {
            return layerGrid(i, j, k) >= 1.0 || layerGrid(i, j - 1, k) >= 1.0;
        }
        else {
            return layerGrid(i, j, k) >= 1.0;
        }
    }

    inline bool _isFaceVelocityExtrapolatedW(int i, int j, int k) {
        if (k == k_voxels) {
            return layerGrid(i, j, k - 1) >= 1.0;
        }
        else if (k > 0) {
            return layerGrid(i, j, k) >= 1.0 || layerGrid(i, j, k - 1) >= 1.0;
        }
        else {
            return layerGrid(i, j, k) >= 1.0;
        }
    }

    inline bool _isCellIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < i_voxels && j < j_voxels && k < k_voxels;
    }
    inline bool _isPositionInGrid(double x, double y, double z) {
        return x >= 0 && y >= 0 && z >= 0 && x <= dx*i_voxels && y <= dx*j_voxels && z <= dx*k_voxels;
    }

    inline double _randomFloat(double min, double max) {
        return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
    }

    void _positionToGridIndex(double x, double y, double z, int *i, int *j, int *k);

    bool _isSimulationInitialized = false;
    bool _isSimulationRunning = false;
    bool _isFluidInSimulation = false;

    int _currentFrame = 0;

    double dx = 0.1;
    double density = 10.0;
    int i_voxels = 10;
    int j_voxels = 10;
    int k_voxels = 10;

    double CFLConditionNumber = 2.0;
    double minTimeStep = 1.0 / 1200.0;
    double maxTimeStep = 1.0 / 30.0;

    glm::vec3 bodyForce;

    MACVelocityField MACVelocity;
    Array3d<int> materialGrid;
    Array3d<double> pressureGrid;
    Array3d<int> layerGrid;

    ImplicitField implicitFluidField;

    std::vector<MarkerParticle> markerParticles;
    std::vector<GridIndex> fluidCellIndices;
};

