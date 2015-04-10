#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>

#include <gl\glew.h>
#include <SDL_opengl.h>
#include <gl\glu.h>

#include "MACVelocityField.h"
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
    void getGridDimensions(int *i, int *j, int *k) { *i = i_voxels; 
                                                     *j = j_voxels; 
                                                     *k = k_voxels; }

private:
    double _calculateNextTimeStep();
    void _advectVelocityField(double dt);

    glm::vec3 _RK2(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK3(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK4(glm::vec3 p0, glm::vec3 v0, double dt);

    double dx = 0.1;
    int i_voxels = 10;
    int j_voxels = 10;
    int k_voxels = 10;

    double CFLConditionNumber = 5.0;
    double minTimeStep = 1.0 / 720.0;
    double maxTimeStep = 1.0 / 30.0;

    MACVelocityField MACVelocity;



};

