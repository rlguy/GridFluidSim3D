#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>

#include <gl\glew.h>
#include <SDL_opengl.h>
#include <gl\glu.h>
#include <assert.h>

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

    void addImplicitFluidPoint(double x, double y, double z, double r) {
        addImplicitFluidPoint(glm::vec3(x, y, z), r);
    }
    void addImplicitFluidPoint(glm::vec3 p, double radius);

    void gridIndexToPosition(int i, int j, int k, double *x, double *y, double *z);
    void gridIndexToCellCenter(int i, int j, int k, double *x, double *y, double *z);

private:
    int M_AIR = 0;
    int M_FLUID = 1;
    int M_SOLID = 2;

    void _initializeSimulation();
    void _initializeMaterialGrid();
    void _initializeFluidMaterial();

    double _calculateNextTimeStep();
    void _advectVelocityField(double dt);

    glm::vec3 _RK2(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK3(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK4(glm::vec3 p0, glm::vec3 v0, double dt);

    inline bool _isCellAir(int i, int j, int k) { return materialGrid(i, j, k) == M_AIR; }
    inline bool _isCellFluid(int i, int j, int k) { return materialGrid(i, j, k) == M_FLUID; }
    inline bool _isCellSolid(int i, int j, int k) { return materialGrid(i, j, k) == M_SOLID; }

    inline bool _isFaceBorderingFluidU(int i, int j, int k) {
        if (i > 0) {
            return materialGrid(i, j, k) == M_FLUID || materialGrid(i-1, j, k) == M_FLUID;
        } else {
            return materialGrid(i, j, k) == M_FLUID;
        }
    }

    inline bool _isFaceBorderingFluidV(int i, int j, int k) {
        if (j > 0) {
            return materialGrid(i, j, k) == M_FLUID || materialGrid(i, j-1, k) == M_FLUID;
        } else {
            return materialGrid(i, j, k) == M_FLUID;
        }
    }

    inline bool _isFaceBorderingFluidW(int i, int j, int k) {
        if (k > 0) {
            return materialGrid(i, j, k) == M_FLUID || materialGrid(i, j, k-1) == M_FLUID;
        } else {
            return materialGrid(i, j, k) == M_FLUID;
        }
    }

    inline bool _isCellIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < i_voxels && j < j_voxels && k < k_voxels;
    }
    inline bool _isPositionInGrid(double x, double y, double z) {
        return x >= 0 && y >= 0 && z >= 0 && x <= dx*i_voxels && y <= dx*j_voxels && z <= dx*k_voxels;
    }

    bool _isSimulationInitialized = false;
    bool _isSimulationRunning = false;
    bool _isFluidInSimulation = false;

    double dx = 0.1;
    int i_voxels = 10;
    int j_voxels = 10;
    int k_voxels = 10;

    double CFLConditionNumber = 5.0;
    double minTimeStep = 1.0 / 720.0;
    double maxTimeStep = 1.0 / 30.0;

    MACVelocityField MACVelocity;
    Array3d<int> materialGrid;
    Array3d<double> pressureGrid;

    ImplicitField implicitFluidField;
};

