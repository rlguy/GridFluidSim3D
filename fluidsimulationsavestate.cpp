#include "fluidsimulationsavestate.h"
#include "fluidsimulation.h"


FluidSimulationSaveState::FluidSimulationSaveState() {
}


FluidSimulationSaveState::~FluidSimulationSaveState() {
}

void FluidSimulationSaveState::saveState(std::string filename, FluidSimulation *_fluidsim) {

    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream state(filename.c_str(), std::ios::out | std::ios::binary);

    int i, j, k;
    _fluidsim->getGridDimensions(&i, &j, &k);
    double dx = _fluidsim->getCellSize();
    _width = i;
    _height = j;
    _depth = k;

    // i, j, k, dx
    _writeInt(&i, &state);
    _writeInt(&j, &state);
    _writeInt(&k, &state);
    _writeDouble(&dx, &state);

    // number of marker particles
    std::vector<glm::vec3> markerParticles = _fluidsim->getMarkerParticles();
    int n = markerParticles.size();
    _writeInt(&n, &state);

    // next frame to be processed
    int currentFrame = _fluidsim->getCurrentFrame();
    _writeInt(&currentFrame, &state);

    // number of solid cell indices
    int numIndices = _getNumSolidCells(_fluidsim);
    _writeInt(&numIndices, &state);

    // floats: marker particle positions in form [x1, y1, z1, x2, y2, z2, ...]
    _writeBinaryMarkerParticlePositions(markerParticles, &state);

    // doubles: MACVelocityGrid U, V, W values ordered each as a flattened 3d array
    MACVelocityField *vfield = _fluidsim->getVelocityField();
    _writeBinaryVelocitiesU(vfield, &state);
    _writeBinaryVelocitiesV(vfield, &state);
    _writeBinaryVelocitiesW(vfield, &state);

    // ints: solid cell indicies in form [i1, j1, k1, i2, j2, k2, ...]
    _writeSolidCellIndices(_fluidsim, &state);

    state.close();
}

bool FluidSimulationSaveState::loadState(std::string filename) {
    _stateData = LoadStateData();

    std::ifstream state(filename.c_str(), std::ios::in | std::ios::binary);

    int numMarkerParticles, numSolidCellIndices;

    bool success = _readInt(&_stateData.i, &state) &&
                   _readInt(&_stateData.j, &state) &&
                   _readInt(&_stateData.k, &state) &&
                   _readDouble(&_stateData.dx, &state) &&
                   _readInt(&numMarkerParticles, &state) &&
                   _readInt(&_stateData.currentFrame, &state) &&
                   _readInt(&numSolidCellIndices, &state);

    if (!success) {
        return false;
    }

    success = _readMarkerParticles(_stateData.markerParticles, numMarkerParticles, &state);
    if (!success) {
        return false;
    }

    _stateData.vgrid = MACVelocity(_stateData.i, _stateData.j, _stateData.k);
    success = _readMACVelocityGrid(_stateData.vgrid, &state);
    if (!success) {
        return false;
    }

    success = _readSolidCellIndices(_stateData.solidCellIndices, numSolidCellIndices, &state);
    if (!success) {
        return false;
    }

    _isLoadStateInitialized = true;
    return true;
}

void FluidSimulationSaveState::getGridDimensions(int *i, int *j, int *k) {
    assert(_isLoadStateInitialized);
    *i = _stateData.i;
    *j = _stateData.j;
    *k = _stateData.k;
}

double FluidSimulationSaveState::getCellSize() {
    assert(_isLoadStateInitialized);
    return _stateData.dx;
}

int FluidSimulationSaveState::getCurrentFrame() {
    assert(_isLoadStateInitialized);
    return _stateData.currentFrame;
}

std::vector<glm::vec3> FluidSimulationSaveState::getMarkerParticles() {
    assert(_isLoadStateInitialized);
    return _stateData.markerParticles;
}

void FluidSimulationSaveState::getVelocityField(Array3d<float> &U, 
                                                Array3d<float> &V, 
                                                Array3d<float> &W) {
    assert(_isLoadStateInitialized);
    U = _stateData.vgrid.U;
    V = _stateData.vgrid.V;
    W = _stateData.vgrid.W;
}

std::vector<GridIndex> FluidSimulationSaveState::getSolidCellIndices() {
    assert(_isLoadStateInitialized);
    return _stateData.solidCellIndices;
}

bool FluidSimulationSaveState::isLoadStateInitialized() {
    return _isLoadStateInitialized;
}

void FluidSimulationSaveState::_writeBinaryMarkerParticlePositions(std::vector<glm::vec3> &mps,
                                                                   std::ofstream *state) {
    int binsize = 3 * mps.size() * sizeof(float);
    char *storage = new char[binsize];

    int fsize = (int)sizeof(float);
    int offset = 0;
    char position[3*sizeof(float)];

    glm::vec3 mp;
    for (unsigned int i = 0; i < mps.size(); i++) {
        mp = mps[i];

        memcpy(position, &mp.x, fsize);
        memcpy(position + fsize, &mp.y, fsize);
        memcpy(position + 2*fsize, &mp.z, fsize);
        memcpy(storage + offset, position, 3*fsize);

        offset += 3*fsize;
    }

    state->write(storage, binsize);
    delete[] storage;
}

void FluidSimulationSaveState::_writeInt(int *value, std::ofstream *state) {
    char bin[sizeof(int)];
    memcpy(bin, value, sizeof(int));
    state->write(bin, sizeof(int));
}

void FluidSimulationSaveState::_writeDouble(double *value, std::ofstream *state) {
    char bin[sizeof(double)];
    memcpy(bin, value, sizeof(double));
    state->write(bin, sizeof(double));
}


void FluidSimulationSaveState::_writeBinaryVelocitiesU(MACVelocityField *vgrid, 
                                                     std::ofstream *state) {
    int binsize = (_width + 1)*_height*_depth*sizeof(float);
    char *storage = new char[binsize];
    float *u = vgrid->getRawArrayU();
    memcpy(storage, u, binsize);
    state->write(storage, binsize);
    delete[] storage;
}

void FluidSimulationSaveState::_writeBinaryVelocitiesV(MACVelocityField *vgrid, 
                                                       std::ofstream *state) {
    int binsize = _width*(_height + 1)*_depth*sizeof(float);
    char *storage = new char[binsize];
    float *v = vgrid->getRawArrayV();
    memcpy(storage, v, binsize);
    state->write(storage, binsize);
    delete[] storage;
}

void FluidSimulationSaveState::_writeBinaryVelocitiesW(MACVelocityField *vgrid, 
                                                       std::ofstream *state) {
    int binsize = _width*_height*(_depth + 1)*sizeof(float);
    char *storage = new char[binsize];
    float *w = vgrid->getRawArrayW();
    memcpy(storage, w, binsize);
    state->write(storage, binsize);
    delete[] storage;
}

int FluidSimulationSaveState::_getNumSolidCells(FluidSimulation *sim) {
    int count = 0;
    for (int k = 0; k < _depth; k++) {
        for (int j = 0; j < _height; j++) {
            for (int i = 0; i < _width; i++) {
                if (sim->getMaterial(i, j, k) == M_SOLID) {
                    count++;
                }
            }
        }
    }

    return count;
}

void FluidSimulationSaveState::_writeSolidCellIndices(FluidSimulation *sim, 
                                                      std::ofstream *state) {
    std::vector<GridIndex> indices;
    for (int k = 0; k < _depth; k++) {
        for (int j = 0; j < _height; j++) {
            for (int i = 0; i < _width; i++) {
                if (sim->getMaterial(i, j, k) == M_SOLID) {
                    indices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

    int *data = new int[3*indices.size()];
    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        data[3*i] = g.i;
        data[3*i + 1] = g.j;
        data[3*i + 2] = g.k;
    }

    int binsize = 3*indices.size()*sizeof(int);
    char *storage = new char[binsize];
    memcpy(storage, data, binsize);
    delete[] data;
    state->write(storage, binsize);

    delete[] storage;
}

bool FluidSimulationSaveState::_readInt(int *value, std::ifstream *state) {
    char bin[sizeof(int)];
    state->read(bin, sizeof(int));

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(int));

    return true;
}

bool FluidSimulationSaveState::_readDouble(double *value, std::ifstream *state) {
    char bin[sizeof(double)];
    state->read(bin, sizeof(double));

    if (!state->good()) {
        return false;
    }

    memcpy(value, bin, sizeof(double));

    return true;
}

bool FluidSimulationSaveState::_readMarkerParticles(std::vector<glm::vec3> &particles, 
                                                    int numParticles,
                                                    std::ifstream *state) {
    int binsize = 3*numParticles*sizeof(float);
    char *bin = new char[binsize];

    state->read(bin, binsize);

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    float *positions = new float[3*numParticles];
    memcpy(positions, bin, binsize);
    delete[] bin;
    for (int i = 0; i < 3*numParticles; i += 3) {
        float x = positions[i];
        float y = positions[i + 1];
        float z = positions[i + 2];

        particles.push_back(glm::vec3(x, y, z));
    }
    delete[] positions;

    return true;
}

bool FluidSimulationSaveState::_readMACVelocityGrid(MACVelocity &vgrid, 
                                                    std::ifstream *state) {
    int isize = vgrid.width;
    int jsize = vgrid.height;
    int ksize = vgrid.depth;
    int binsize = ((isize+1)*jsize*ksize + 
                    isize*(jsize+1)*ksize + 
                    isize*jsize*(ksize+1)) * sizeof(double);

    char *bin = new char[binsize];
    state->read(bin, binsize);

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    int dsize = sizeof(double);
    int offset = 0;
    int usize = (isize+1)*jsize*ksize*dsize;
    double *uarray = new double[usize];
    memcpy(uarray, bin, usize);
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize + 1; i++) {
                vgrid.U.set(i, j, k, uarray[offset]);
                offset++;
            }
        }
    }
    delete[] uarray;

    offset = 0;
    int vsize = isize*(jsize+1)*ksize*dsize;
    double *varray = new double[vsize];
    memcpy(varray, bin + usize, vsize);
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize + 1; j++) {
            for (int i = 0; i < isize; i++) {
                vgrid.V.set(i, j, k, varray[offset]);
                offset++;
            }
        }
    }
    delete[] varray;

    offset = 0;
    int wsize = isize*jsize*(ksize+1)*dsize;
    double *warray = new double[wsize];
    memcpy(warray, bin + usize + vsize, wsize);
    delete[] bin;
    for (int k = 0; k < ksize + 1; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                vgrid.W.set(i, j, k, warray[offset]);
                offset++;
            }
        }
    }
    delete[] warray;

    return true;
}

bool FluidSimulationSaveState::_readSolidCellIndices(std::vector<GridIndex> &indices, 
                                                     int numIndices,
                                                     std::ifstream *state) {

    int binsize = 3*numIndices*sizeof(int);
    char *bin = new char[binsize];

    state->read(bin, binsize);

    if (!state->good()) {
        delete[] bin;
        return false;
    }

    int *data = new int[3*numIndices];
    memcpy(data, bin, binsize);
    delete[] bin;
    for (int idx = 0; idx < 3*numIndices; idx += 3) {
        int i = data[idx];
        int j = data[idx + 1];
        int k = data[idx + 2];

        indices.push_back(GridIndex(i, j, k));
    }
    delete[] data;

    return true;
}