#include "fluidsimulation.h"


FluidSimulation::FluidSimulation() :
                                bodyForce(glm::vec3(0.0, 0.0, 0.0)),
                                MACVelocity(i_voxels, j_voxels, k_voxels, dx),
                                materialGrid(Array3d<int>(i_voxels, j_voxels, k_voxels, M_AIR)),
                                pressureGrid(Array3d<double>(i_voxels, j_voxels, k_voxels, 0.0)),
                                layerGrid(Array3d<int>(i_voxels, j_voxels, k_voxels, -1))
{
}

FluidSimulation::FluidSimulation(int x_voxels, int y_voxels, int z_voxels, double cell_size) : 
                                 i_voxels(x_voxels), j_voxels(y_voxels), k_voxels(z_voxels), dx(cell_size), 
                                 bodyForce(glm::vec3(0.0, 0.0, 0.0)),
                                 MACVelocity(x_voxels, y_voxels, z_voxels, cell_size),
                                 materialGrid(Array3d<int>(i_voxels, j_voxels, k_voxels, M_AIR)),
                                 pressureGrid(Array3d<double>(x_voxels, y_voxels, z_voxels, 0.0)),
                                 layerGrid(Array3d<int>(x_voxels, y_voxels, z_voxels, -1)),
                                 implicitFluidField(x_voxels*cell_size, y_voxels*cell_size, z_voxels*cell_size)
{
}

FluidSimulation::~FluidSimulation() {

}

void FluidSimulation::run() {
    if (!_isSimulationInitialized) {
        _initializeSimulation();
    }
    _isSimulationRunning = true;
}

void FluidSimulation::pause() {
    if (!_isSimulationInitialized) {
        return;
    }

    _isSimulationRunning = !_isSimulationRunning;
}

void FluidSimulation::addBodyForce(glm::vec3 f) {
    bodyForce += f;
}

void FluidSimulation::setBodyForce(glm::vec3 f) {
    bodyForce = f;
}

void FluidSimulation::addImplicitFluidPoint(glm::vec3 p, double r) {
    implicitFluidField.addPoint(p, r);
}

std::vector<ImplicitPointData> FluidSimulation::getImplicitFluidPoints() {
    return implicitFluidField.getImplicitPointData();
}

std::vector<glm::vec3> FluidSimulation::getMarkerParticles() {
    std::vector<glm::vec3> particles;

    for (int i = 0; i < (int)markerParticles.size(); i++) {
        particles.push_back(markerParticles[i].position);
    }

    return particles;
}

void FluidSimulation::_initializeSolidCells() {
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

void FluidSimulation::_addMarkerParticlesToCell(int i, int j, int k) {
    double q = 0.25*dx;
    double cx, cy, cz;
    gridIndexToCellCenter(i, j, k, &cx, &cy, &cz);

    glm::vec3 points[] = {
        glm::vec3(cx - q, cy - q, cz - q),
        glm::vec3(cx + q, cy - q, cz - q),
        glm::vec3(cx + q, cy - q, cz + q),
        glm::vec3(cx - q, cy - q, cz + q),
        glm::vec3(cx - q, cy + q, cz - q),
        glm::vec3(cx + q, cy + q, cz - q),
        glm::vec3(cx + q, cy + q, cz + q),
        glm::vec3(cx - q, cy + q, cz + q)
    };

    double eps = 10e-6;
    double jitter = 0.125*dx - eps;

    for (int idx = 0; idx < 8; idx++) {
        glm::vec3 jit = glm::vec3(_randomFloat(-jitter, jitter),
                                  _randomFloat(-jitter, jitter),
                                  _randomFloat(-jitter, jitter));

        glm::vec3 p = points[idx] + jit;
        markerParticles.push_back(MarkerParticle(p, i, j, k));
    }
}

void FluidSimulation::_initializeFluidMaterial() {
    _isFluidInSimulation = implicitFluidField.getNumPoints() > 0;

    if (!_isFluidInSimulation) {
        return;
    }

    for (int k = 0; k < materialGrid.depth; k++) {
        for (int j = 0; j < materialGrid.height; j++) {
            for (int i = 0; i < materialGrid.width; i++) {
                double x, y, z;
                gridIndexToCellCenter(i, j, k, &x, &y, &z);

                if (implicitFluidField.isInside(x, y, z) && _isCellAir(i, j, k)) {
                    materialGrid.set(i, j, k, M_FLUID);
                    _addMarkerParticlesToCell(i, j, k);
                }
            }
        }
    }
}

void FluidSimulation::_initializeSimulation() {
    _initializeSolidCells();
    _initializeFluidMaterial();
    _isSimulationInitialized = true;
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
    // TODO: Update velocities on temporary grid

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

void FluidSimulation::gridIndexToPosition(int i, int j, int k, double *x, double *y, double *z) {
    assert(_isCellIndexInRange(i, j, k));

    *x = (double)i*dx;
    *y = (double)j*dx;
    *z = (double)k*dx;
}

void FluidSimulation::gridIndexToCellCenter(int i, int j, int k, double *x, double *y, double *z) {
    assert(_isCellIndexInRange(i, j, k));

    *x = (double)i*dx + 0.5*dx;
    *y = (double)j*dx + 0.5*dx;
    *z = (double)k*dx + 0.5*dx;
}

void FluidSimulation::positionToGridIndex(double x, double y, double z, int *i, int *j, int *k) {
    assert(_isPositionInGrid(x, y, z));

    double invdx = 1.0 / dx;
    *i = fminf((int)floor(x*invdx), i_voxels);
    *j = fminf((int)floor(y*invdx), j_voxels);
    *k = fminf((int)floor(z*invdx), k_voxels);
}

void FluidSimulation::_updateFluidCells() {
    for (int k = 0; k < materialGrid.depth; k++) {
        for (int j = 0; j < materialGrid.height; j++) {
            for (int i = 0; i < materialGrid.width; i++) {
                if (_isCellFluid(i, j, k)) {
                    materialGrid.set(i, j, k, M_AIR);
                }
            }
        }
    }

    MarkerParticle p;
    for (int i = 0; i < (int)markerParticles.size(); i++) {
        p = markerParticles[i];
        assert(!_isCellSolid(p.i, p.j, p.k));
        materialGrid.set(p.i, p.j, p.k, M_FLUID);
    }

    fluidCellIndices.clear();
    for (int k = 0; k < materialGrid.depth; k++) {
        for (int j = 0; j < materialGrid.height; j++) {
            for (int i = 0; i < materialGrid.width; i++) {
                if (_isCellFluid(i, j, k)) {
                    fluidCellIndices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }
}

void FluidSimulation::_getCellNeighbourGridIndices(int i, int j, int k, GridIndex n[6]) {
    n[0] = GridIndex(i-1, j, k);
    n[1] = GridIndex(i+1, j, k);
    n[2] = GridIndex(i, j-1, k);
    n[3] = GridIndex(i, j+1, k);
    n[4] = GridIndex(i, j, k-1);
    n[5] = GridIndex(i, j, k+1);
}

void FluidSimulation::_updateExtrapolationLayer(int layerIndex) {
    GridIndex neighbours[6];
    GridIndex n;

    for (int k = 0; k < layerGrid.depth; k++) {
        for (int j = 0; j < layerGrid.height; j++) {
            for (int i = 0; i < layerGrid.width; i++) {
                if (layerGrid(i, j, k) == layerIndex - 1 && !_isCellSolid(i, j, k)) {
                    _getCellNeighbourGridIndices(i, j, k, neighbours);
                    for (int idx = 0; idx < 6; idx++) {
                        n = neighbours[idx];
                        if (_isCellIndexInRange(n.i, n.j, n.k) && layerGrid(n.i, n.j, n.k) == -1 && 
                                                                  _isCellAir(n.i, n.j, n.k)) {
                            layerGrid.set(n.i, n.j, n.k, layerIndex);
                        }
                    }
                }
            }
        }
    }
}

int FluidSimulation::_updateExtrapolationLayers() {
    layerGrid.fill(-1);

    GridIndex idx;
    for (int i = 0; i < (int)fluidCellIndices.size(); i++) {
        idx = fluidCellIndices[i];
        layerGrid.set(idx.i, idx.j, idx.k, 0);
    }

    // add 2 extra layers to account for extra values needed during cubic 
    // interpolation calculations
    int numLayers = ceil(CFLConditionNumber) + 2;
    for (int layer = 1; layer <= numLayers; layer++) {
        _updateExtrapolationLayer(layer);
    }

    return numLayers;
}

void FluidSimulation::_extrapolateVelocitiesForLayerIndex(int idx) {
    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels + 1; i++) {
                if (_isFaceBorderingLayerIndexU(i, j, k, idx) &&
                    !_isFaceBorderingLayerIndexU(i, j, k, idx-1)) {

                }
            }
        }
    }

    for (int k = 0; k < k_voxels; k++) {
        for (int j = 0; j < j_voxels + 1; j++) {
            for (int i = 0; i < i_voxels; i++) {
                if (_isFaceBorderingLayerIndexV(i, j, k, idx) &&
                    !_isFaceBorderingLayerIndexV(i, j, k, idx-1)) {

                }
            }
        }
    }

    for (int k = 0; k < k_voxels + 1; k++) {
        for (int j = 0; j < j_voxels; j++) {
            for (int i = 0; i < i_voxels; i++) {
                if (_isFaceBorderingLayerIndexW(i, j, k, idx) &&
                    !_isFaceBorderingLayerIndexW(i, j, k, idx-1)) {

                }
            }
        }
    }
}

void FluidSimulation::_extrapolateFluidVelocities() {
    int numLayers = _updateExtrapolationLayers();

    for (int i = 1; i <= numLayers; i++) {
        _extrapolateVelocitiesForLayerIndex(i);
    }
}

void FluidSimulation::_stepFluid(double dt) {
    _updateFluidCells();
    _extrapolateFluidVelocities();
}

void FluidSimulation::update(double dt) {
    if (!_isSimulationRunning || !_isSimulationInitialized || !_isFluidInSimulation) {
        return;
    }

    int numsteps = 0;
    double timeleft = dt;
    while (timeleft > 0.0) {
        double timestep = _calculateNextTimeStep();
        if (timeleft - timestep < 0.0) {
            timestep = timeleft;
        }
        timeleft -= timestep;
        numsteps++;
        _stepFluid(timestep);
    }

    _currentFrame++;
}

void FluidSimulation::draw() {


}