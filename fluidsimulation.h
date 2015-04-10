#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>

#include <gl\glew.h>
#include <SDL_opengl.h>
#include <gl\glu.h>

#include "MACVelocityField.h"
#include "array3d.h"
#include "glm/glm.hpp"

class FluidSimulation
{
public:
    FluidSimulation();
    FluidSimulation(int x_voxels, int y_voxels, int z_voxels, double cell_size);
    ~FluidSimulation();

    void update(double dt);
    void draw();

    double getCellSize() { return dx; }
    void getGridDimensions(int *i, int *j, int *k) { *i = i_voxels; *j = j_voxels; *k = k_voxels; }
    int getMaterial(int i, int j, int k) { return materialGrid(i, j, k); }

private:
    int M_AIR = 0;
    int M_FLUID = 1;
    int M_SOLID = 2;

    void _initMaterialGrid();

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


};

