#include "fluidsimulation.h"


FluidSimulation::FluidSimulation() {
    MACVelocity = MACVelocityField(i_voxels, j_voxels, k_voxels, dx);
    _initMaterialGrid();
    pressureGrid = Array3d<double>(i_voxels, j_voxels, k_voxels);
    pressureGrid.fill(0.0);
}

FluidSimulation::FluidSimulation(int x_voxels, int y_voxels, int z_voxels, double cell_size) : 
                                 i_voxels(x_voxels), j_voxels(y_voxels), k_voxels(z_voxels),
                                 dx(cell_size), MACVelocity(x_voxels, y_voxels, z_voxels, cell_size) {
    MACVelocity.randomizeValues(0.0, 20.0);

    _initMaterialGrid();
    pressureGrid = Array3d<double>(i_voxels, j_voxels, k_voxels);
    pressureGrid.fill(0.0);
}

FluidSimulation::~FluidSimulation() {

}

void FluidSimulation::_initMaterialGrid() {
    materialGrid = Array3d<int>(i_voxels, j_voxels, k_voxels);
    materialGrid.fill(M_AIR);

    // fill borders with solid cells
    for (int j = 0; j < j_voxels; j++) {
        for (int i = 0; i < i_voxels; i++) {
            materialGrid.set(i, j, 0, M_SOLID);
            materialGrid.set(i, j, k_voxels-1, M_SOLID);
        }
    }

    for (int k = 0; k < k_voxels; k++) {
        for (int i = 0; i < i_voxels; i++) {
            materialGrid.set(i, 0, k, M_SOLID);
            materialGrid.set(i, j_voxels-1, k, M_SOLID);
        }
    }

    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels; j++) {
            materialGrid.set(0, j, k, M_SOLID);
            materialGrid.set(i_voxels-1, j, k, M_SOLID);
        }
    }
}

glm::vec3 FluidSimulation::_RK2(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 p1 = p0 + (float)dt*k2;

    return p1;
}

glm::vec3 FluidSimulation::_RK3(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 k3 = MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.75*dt)*k2);
    glm::vec3 p1 = p0 + (float)(dt/9.0f)*(2.0f*k1 + 3.0f*k2 + 4.0f*k3);

    return p1;
}

glm::vec3 FluidSimulation::_RK4(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 k3 = MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k2);
    glm::vec3 k4 = MACVelocity.evaluateVelocityAtPosition(p0 + (float)dt*k3);
    glm::vec3 p1 = p0 + (float)(dt/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);

    return p1;
}

double FluidSimulation::_calculateNextTimeStep() {
    double maxu = MACVelocity.evaluateMaximumVelocityMagnitude();
    double timeStep = CFLConditionNumber*dx / maxu;

    timeStep = fmaxf(minTimeStep, timeStep);
    timeStep = fminf(maxTimeStep, timeStep);

    return timeStep;
}

void FluidSimulation::_advectVelocityField(double dt) {

    glm::vec3 p0, p1, v0, v1;

    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels + 1; i++) {
               
                // TODO: advect only fluid velocities

                p0 = MACVelocity.velocityIndexToPositionU(i, j, k);
                v0 = MACVelocity.evaluateVelocityAtFaceCenterU(i, j, k);
                p1 = _RK4(p0, v0, -dt);

                // TODO: handle boundary conditions for p1

                v1 = MACVelocity.evaluateVelocityAtPosition(p1);
                MACVelocity.setU(i, j, k, v1.x);

            }
        }
    }

    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels + 1; j++) {
            for (int i = 0; i < i_voxels; i++) {
                
                // TODO: advect only fluid velocities

                p0 = MACVelocity.velocityIndexToPositionV(i, j, k);
                v0 = MACVelocity.evaluateVelocityAtFaceCenterV(i, j, k);
                p1 = _RK4(p0, v0, -dt);

                // TODO: handle boundary conditions for p1

                v1 = MACVelocity.evaluateVelocityAtPosition(p1);
                MACVelocity.setV(i, j, k, v1.y);

            }
        }
    }

    for (int k = 0; k < k_voxels + 1; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels; i++) {
                
                // TODO: advect only fluid velocities

                p0 = MACVelocity.velocityIndexToPositionW(i, j, k);
                v0 = MACVelocity.evaluateVelocityAtFaceCenterW(i, j, k);
                p1 = _RK4(p0, v0, -dt);

                // TODO: handle boundary conditions for p1

                v1 = MACVelocity.evaluateVelocityAtPosition(p1);
                MACVelocity.setW(i, j, k, v1.z);

            }
        }
    }

}

void FluidSimulation::update(double dt) {
}

void FluidSimulation::draw() {


}