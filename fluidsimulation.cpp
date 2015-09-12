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
#include "fluidsimulation.h"


FluidSimulation::FluidSimulation() {
}

FluidSimulation::FluidSimulation(int x_voxels, int y_voxels, int z_voxels, double cell_size) :
                                _isize(x_voxels), _jsize(y_voxels), _ksize(z_voxels), _dx(cell_size),
                                _bodyForce(0.0, 0.0, 0.0),
                                _MACVelocity(_isize, _jsize, _ksize, cell_size),
                                _materialGrid(Array3d<int>(_isize, _jsize, _ksize, M_AIR)),
                                _levelset(_isize, _jsize, _ksize, cell_size),
                                _markerParticleRadius(pow(3*(_dx*_dx*_dx / 8.0) / (4*3.141592653), 1.0/3.0))
{
    _materialGrid.setOutOfRangeValue(M_SOLID);
    _logfile = LogFile();
}

FluidSimulation::FluidSimulation(FluidSimulationSaveState &state) {
    assert(state.isLoadStateInitialized());
    _initializeSimulationFromSaveState(state);

    _materialGrid.setOutOfRangeValue(M_SOLID);
    _logfile = LogFile();
}

FluidSimulation::~FluidSimulation() {
    for (unsigned int i = 0; i < _fluidSources.size(); i++) {
        delete[] _fluidSources[i];
    }
}

/*******************************************************************************
    PUBLIC
********************************************************************************/

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

void FluidSimulation::saveState() {
    saveState("savestates/autosave.state");
}

void FluidSimulation::saveState(std::string filename) {
    FluidSimulationSaveState state = FluidSimulationSaveState();
    state.saveState(filename, this);
}

int FluidSimulation::getCurrentFrame() {
    return _currentFrame;
}

bool FluidSimulation::isCurrentFrameFinished() { 
    return _isCurrentFrameFinished; 
}

double FluidSimulation::getCellSize() { 
    return _dx; 
}

void FluidSimulation::getGridDimensions(int *i, int *j, int *k) { 
    *i = _isize; *j = _jsize; *k = _ksize; 
}

void FluidSimulation::getSimulationDimensions(double *w, double *h, double *d) { 
    *w = (double)_isize*_dx;
    *h = (double)_jsize*_dx;
    *d = (double)_ksize*_dx;
}

double FluidSimulation::getSimulationWidth() {  
    return (double)_isize*_dx; 
}

double FluidSimulation::getSimulationHeight() { 
    return (double)_jsize*_dx; 
}

double FluidSimulation::getSimulationDepth() {  
    return (double)_ksize*_dx; 
}

double FluidSimulation::getDensity() { 
    return _density; 
}

void FluidSimulation::setDensity(double p) { 
    assert(p > 0); _density = p; 
}

int FluidSimulation::getMaterial(int i, int j, int k) { 
    return _materialGrid(i, j, k); 
}

void FluidSimulation::setMarkerParticleScale(double s) { 
    _markerParticleScale = s; 
}


void FluidSimulation::setSurfaceSubdivisionLevel(unsigned int n) {
    assert(n >= 1);
    _outputFluidSurfaceSubdivisionLevel = n;
}

void FluidSimulation::enableSurfaceMeshOutput() {
    _isSurfaceMeshOutputEnabled = true;
}

void FluidSimulation::disableSurfaceMeshOutput() {
    _isSurfaceMeshOutputEnabled = false;
}

void FluidSimulation::enableDiffuseMaterialOutput() {
    _isDiffuseMaterialOutputEnabled = true;
}

void FluidSimulation::disableDiffuseMaterialOutput() {
    _isDiffuseMaterialOutputEnabled = false;
}

void FluidSimulation::enableBrickOutput() {
    AABB brick = AABB(glm::vec3(), _brickWidth, _brickHeight, _brickDepth);
    _fluidBrickGrid = FluidBrickGrid(_isize, _jsize, _ksize, _dx, brick);
    _isBrickOutputEnabled = true;
}

void FluidSimulation::enableBrickOutput(double width, double height, double depth) {
    assert(width > 0.0 && height > 0.0 && depth > 0.0);
    _brickWidth = width;
    _brickHeight = height;
    _brickDepth = depth;

    AABB brick = AABB(glm::vec3(), _brickWidth, _brickHeight, _brickDepth);

    int i, j, k;
    _fluidBrickGrid.getGridDimensions(&i, &j, &k);
    if (i != _isize || j != _jsize || k != _ksize) {
        _fluidBrickGrid = FluidBrickGrid(_isize, _jsize, _ksize, _dx, brick);
    }
    _fluidBrickGrid.setBrickDimensions(brick);

    _isBrickOutputEnabled = true;
}

void FluidSimulation::disableBrickOutput() {
    _isBrickOutputEnabled = false;
}

void FluidSimulation::addBodyForce(double fx, double fy, double fz) { 
    addBodyForce(glm::vec3(fx, fy, fz)); 
}

void FluidSimulation::addBodyForce(glm::vec3 f) {
    _bodyForce += f;
}

void FluidSimulation::setBodyForce(double fx, double fy, double fz) { 
    setBodyForce(glm::vec3(fx, fy, fz)); 
}

void FluidSimulation::setBodyForce(glm::vec3 f) {
    _bodyForce = f;
}

void FluidSimulation::addImplicitFluidPoint(double x, double y, double z, double r) {
    addImplicitFluidPoint(glm::vec3(x, y, z), r);
}

void FluidSimulation::addImplicitFluidPoint(glm::vec3 p, double r) {
    if (_fluidInitializationType == MESH) {
        return;
    }

    _fluidPoints.push_back(FluidPoint(p, r));
    _fluidInitializationType = IMPLICIT;
}

void FluidSimulation::addFluidCuboid(double x, double y, double z, 
                                     double w, double h, double d) {
    addFluidCuboid(glm::vec3(x, y, z), w, h, d);
}

void FluidSimulation::addFluidCuboid(glm::vec3 p1, glm::vec3 p2) {
    if (_fluidInitializationType == MESH) {
        return;
    }

    glm::vec3 minp = glm::vec3(fmin(p1.x, p2.x),
                               fmin(p1.y, p2.y), 
                               fmin(p1.z, p2.z));
    double width = fabs(p2.x - p1.x);
    double height = fabs(p2.y - p1.y);
    double depth = fabs(p2.z - p1.z);

    addFluidCuboid(minp, width, height, depth);
    _fluidInitializationType = IMPLICIT;
}

void FluidSimulation::addFluidCuboid(glm::vec3 p, double w, double h, double d) {
    if (_fluidInitializationType == MESH) {
        return;
    }

    _fluidCuboids.push_back(FluidCuboid(p, w, h, d));
    _fluidInitializationType = IMPLICIT;
}

bool FluidSimulation::addFluidMesh(std::string OBJFilename) {
    return addFluidMesh(OBJFilename, glm::vec3(0.0, 0.0, 0.0), 1.0);
}

bool FluidSimulation::addFluidMesh(std::string OBJFilename, glm::vec3 offset) {
    return addFluidMesh(OBJFilename, offset, 1.0);
}

bool FluidSimulation::addFluidMesh(std::string OBJFilename, double scale) {
    return addFluidMesh(OBJFilename, glm::vec3(0.0, 0.0, 0.0), scale);
}

bool FluidSimulation::addFluidMesh(std::string OBJFilename, glm::vec3 offset, double scale) {
    if (_fluidInitializationType == IMPLICIT) {
        return false;
    }
    _fluidMeshFilename = OBJFilename;
    _fluidMeshOffset = offset;
    _fluidMeshScale = scale;
    _fluidInitializationType = MESH;

    return true;
}

SphericalFluidSource* FluidSimulation::addSphericalFluidSource(glm::vec3 pos, double r) {
    SphericalFluidSource *source = new SphericalFluidSource(pos, r);
    source->setID(_getUniqueFluidSourceID());

    _fluidSources.push_back(source);
    _sphericalFluidSources.push_back(source);
    return source;
}

SphericalFluidSource* FluidSimulation::addSphericalFluidSource(glm::vec3 pos, double r, 
                                             glm::vec3 velocity) {
    SphericalFluidSource *source = new SphericalFluidSource(pos, r, velocity);
    source->setID(_getUniqueFluidSourceID());

    _fluidSources.push_back(source);
    _sphericalFluidSources.push_back(source);
    return source;
}

CuboidFluidSource* FluidSimulation::addCuboidFluidSource(AABB bbox) {
    CuboidFluidSource *source = new CuboidFluidSource(bbox);
    source->setID(_getUniqueFluidSourceID());

    _fluidSources.push_back(source);
    _cuboidFluidSources.push_back(source);
    return source;
}

CuboidFluidSource* FluidSimulation::addCuboidFluidSource(AABB bbox, glm::vec3 velocity) {
    CuboidFluidSource *source = new CuboidFluidSource(bbox, velocity);
    source->setID(_getUniqueFluidSourceID());

    _fluidSources.push_back(source);
    _cuboidFluidSources.push_back(source);
    return source;
}

void FluidSimulation::removeFluidSource(FluidSource *source) {
    bool isFound = false;
    for (unsigned int i = 0; i < _fluidSources.size(); i++) {
        if (source->getID() == _fluidSources[i]->getID()) {
            delete (_fluidSources[i]);
            _fluidSources.erase(_fluidSources.begin() + i);
            isFound = true;
            break;
        }
    }

    assert(isFound);

    isFound = false;
    for (unsigned int i = 0; i < _sphericalFluidSources.size(); i++) {
        if (source->getID() == _sphericalFluidSources[i]->getID()) {
            _sphericalFluidSources.erase(_sphericalFluidSources.begin() + i);
            isFound = true;
            break;
        }
    }

    for (unsigned int i = 0; i < _cuboidFluidSources.size(); i++) {
        if (source->getID() == _cuboidFluidSources[i]->getID()) {
            _cuboidFluidSources.erase(_cuboidFluidSources.begin() + i);
            isFound = true;
            break;
        }
    }

    assert(isFound);
}

void FluidSimulation::removeFluidSources() {
    for (unsigned int i = 0; i < _fluidSources.size(); i++) {
        delete (_fluidSources[i]);
    }
    _fluidSources.clear();
    _sphericalFluidSources.clear();
    _cuboidFluidSources.clear();
}

void FluidSimulation::addSolidCell(int i, int j, int k) {
    if (_isSimulationInitialized) { return; }
    assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));
    _materialGrid.set(i, j, k, M_SOLID);
}

void FluidSimulation::addSolidCells(std::vector<glm::vec3> indices) {
    for (unsigned int i = 0; i < indices.size(); i++) {
        addSolidCell((int)indices[i].x, (int)indices[i].y, (int)indices[i].z);
    }
}

void FluidSimulation::removeSolidCell(int i, int j, int k) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    // Cannot remove border cells
    if (Grid3d::isGridIndexOnBorder(i, j, k, _isize, _jsize, _ksize)) { 
        return; 
    }

    if (_isCellSolid(i, j, k)) {
        _materialGrid.set(i, j, k, M_AIR);
    }
}

void FluidSimulation::removeSolidCells(std::vector<glm::vec3> indices) {
    for (unsigned int i = 0; i < indices.size(); i++) {
        removeSolidCell((int)indices[i].x, (int)indices[i].y, (int)indices[i].z);
    }
}

std::vector<glm::vec3> FluidSimulation::getSolidCells() {
    std::vector<glm::vec3> indices;
    for (int k = 1; k < _materialGrid.depth - 1; k++) {
        for (int j = 1; j < _materialGrid.height - 1; j++) {
            for (int i = 1; i < _materialGrid.width - 1; i++) {
                if (_isCellSolid(i, j, k)) {
                    indices.push_back(glm::vec3(i, j, k));
                }
            }
        }
    }

    return indices;
}

std::vector<glm::vec3> FluidSimulation::getSolidCellPositions() {
    std::vector<glm::vec3> indices;
    for (int k = 1; k < _materialGrid.depth - 1; k++) {
        for (int j = 1; j < _materialGrid.height - 1; j++) {
            for (int i = 1; i < _materialGrid.width - 1; i++) {
                if (_isCellSolid(i, j, k)) {
                    indices.push_back(Grid3d::GridIndexToCellCenter(i, j, k, _dx));
                }
            }
        }
    }

    return indices;
}

unsigned int FluidSimulation::getNumMarkerParticles() {
    return _markerParticles.size();
}

std::vector<glm::vec3> FluidSimulation::getMarkerParticlePositions() {
    std::vector<glm::vec3> particles;
    particles.reserve(_markerParticles.size());

    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        particles.push_back(_markerParticles[i].position);
    }

    return particles;
}

std::vector<glm::vec3> FluidSimulation::getMarkerParticleVelocities() {
    std::vector<glm::vec3> velocities;
    velocities.reserve(_markerParticles.size());

    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        velocities.push_back(_markerParticles[i].velocity);
    }

    return velocities;
}

std::vector<DiffuseParticle> FluidSimulation::getDiffuseParticles() {
    return _diffuseParticles;
}

Array3d<float> FluidSimulation::getDensityGrid() {
    return _fluidBrickGrid.getDensityGrid();
}

MACVelocityField* FluidSimulation::getVelocityField() { 
    return &_MACVelocity; 
}

LevelSet* FluidSimulation::getLevelSet() { 
    return &_levelset; 
};

TriangleMesh* FluidSimulation::getFluidSurfaceTriangles() {
    return &_surfaceMesh;
}

/********************************************************************************
    INITIALIZATION
********************************************************************************/
void FluidSimulation::_initializeSolidCells() {
    // fill borders with solid cells
    for (int j = 0; j < _jsize; j++) {
        for (int i = 0; i < _isize; i++) {
            _materialGrid.set(i, j, 0, M_SOLID);
            _materialGrid.set(i, j, _ksize-1, M_SOLID);
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int i = 0; i < _isize; i++) {
            _materialGrid.set(i, 0, k, M_SOLID);
            _materialGrid.set(i, _jsize-1, k, M_SOLID);
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            _materialGrid.set(0, j, k, M_SOLID);
            _materialGrid.set(_isize-1, j, k, M_SOLID);
        }
    }
}

void FluidSimulation::_addMarkerParticlesToCell(GridIndex g) {
    _addMarkerParticlesToCell(g, glm::vec3(0.0, 0.0, 0.0));
}

void FluidSimulation::_addMarkerParticlesToCell(GridIndex g, glm::vec3 velocity) {
    double q = 0.25*_dx;
    glm::vec3 c = Grid3d::GridIndexToCellCenter(g, _dx);

    glm::vec3 points[] = {
        glm::vec3(c.x - q, c.y - q, c.z - q),
        glm::vec3(c.x + q, c.y - q, c.z - q),
        glm::vec3(c.x + q, c.y - q, c.z + q),
        glm::vec3(c.x - q, c.y - q, c.z + q),
        glm::vec3(c.x - q, c.y + q, c.z - q),
        glm::vec3(c.x + q, c.y + q, c.z - q),
        glm::vec3(c.x + q, c.y + q, c.z + q),
        glm::vec3(c.x - q, c.y + q, c.z + q)
    };

    double eps = 10e-6;
    double jitter = 0.25*_dx - eps;

    for (int idx = 0; idx < 8; idx++) {
        glm::vec3 jit = glm::vec3(_randomFloat(-jitter, jitter),
                                  _randomFloat(-jitter, jitter),
                                  _randomFloat(-jitter, jitter));

        glm::vec3 p = points[idx] + jit;
        _markerParticles.push_back(MarkerParticle(p, velocity));
    }
}

void FluidSimulation::_getInitialFluidCellsFromImplicitSurface(std::vector<GridIndex> &fluidCells) {
    ImplicitSurfaceScalarField field = ImplicitSurfaceScalarField(_isize + 1, _jsize + 1, _ksize + 1, _dx);
    field.enableCellCenterValues();

    FluidPoint fp;
    for (unsigned int i = 0; i < _fluidPoints.size(); i++) {
        fp = _fluidPoints[i];
        field.addPoint(fp.position, fp.radius);
    }

    FluidCuboid fc;
    for (unsigned int i = 0; i < _fluidCuboids.size(); i++) {
        fc = _fluidCuboids[i];
        field.addCuboid(fc.bbox.position, fc.bbox.width, fc.bbox.height, fc.bbox.depth);
    }

    // Polygonizer needs an estimate of cells that are inside the surface
    for (int k = 0; k < _materialGrid.depth; k++) {
        for (int j = 0; j < _materialGrid.height; j++) {
            for (int i = 0; i < _materialGrid.width; i++) {
                GridIndex g(i, j, k);

                if (field.isCellInsideSurface(i, j, k) && _isCellAir(g)) {
                    fluidCells.push_back(g);
                }
            }
        }
    }

    assert(fluidCells.size() > 0);

    Polygonizer3d polygonizer(field);

    field.setMaterialGrid(_materialGrid);
    polygonizer.setInsideCellIndices(fluidCells);
    fluidCells.clear();

    polygonizer.polygonizeSurface();
    _surfaceMesh = polygonizer.getTriangleMesh();
    _surfaceMesh.setGridDimensions(_isize, _jsize, _ksize, _dx);
    _surfaceMesh.getCellsInsideMesh(fluidCells);
}

void FluidSimulation::_getInitialFluidCellsFromTriangleMesh(std::vector<GridIndex> &fluidCells) {
    bool success = _surfaceMesh.loadOBJ(_fluidMeshFilename, _fluidMeshOffset, _fluidMeshScale);
    assert(success);
    _surfaceMesh.setGridDimensions(_isize, _jsize, _ksize, _dx);
    _surfaceMesh.getCellsInsideMesh(fluidCells);

    LevelSetField field = LevelSetField(_isize, _jsize, _ksize, _dx);
    Polygonizer3d levelsetPolygonizer = Polygonizer3d(&field);
    _levelset.setSurfaceMesh(_surfaceMesh);
    _levelset.calculateSignedDistanceField();
    field.setMaterialGrid(_materialGrid);
    field.setSignedDistanceField(_levelset.getSignedDistanceField());
    levelsetPolygonizer.setInsideCellIndices(fluidCells);
    levelsetPolygonizer.polygonizeSurface();

    fluidCells.clear();
    _surfaceMesh = levelsetPolygonizer.getTriangleMesh();
    _surfaceMesh.setGridDimensions(_isize, _jsize, _ksize, _dx);
    _surfaceMesh.getCellsInsideMesh(fluidCells);
}

void FluidSimulation::_initializeFluidMaterial() {
    _isFluidInSimulation = _fluidInitializationType == MESH ||
                           _fluidInitializationType == IMPLICIT;

    if (!_isFluidInSimulation) {
        return;
    }

    std::vector<GridIndex> fluidCells;
    if (_fluidInitializationType == IMPLICIT) {
        _getInitialFluidCellsFromImplicitSurface(fluidCells);
    } else if (_fluidInitializationType == MESH) {
        _getInitialFluidCellsFromTriangleMesh(fluidCells);
    }


    _markerParticles.reserve(8*fluidCells.size());
    GridIndex g;
    std::vector<GridIndex> indices;
    for (unsigned int i = 0; i < fluidCells.size(); i++) {
        g = fluidCells[i];

        if (_isCellAir(g)) {
            _materialGrid.set(g, M_FLUID);
            _addMarkerParticlesToCell(g);
            indices.push_back(g);
        }
    }

    _fluidCellIndices = indices;
}

void FluidSimulation::_initializeSimulation() {
    _initializeSolidCells();
    _initializeFluidMaterial();

    _isSimulationInitialized = true;
}

void FluidSimulation::_initializeFluidMaterialParticlesFromSaveState() {
    MarkerParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);
        assert(!_isCellSolid(g));
        _materialGrid.set(g, M_FLUID);
    }

    _fluidCellIndices.clear();
    for (int k = 0; k < _materialGrid.depth; k++) {
        for (int j = 0; j < _materialGrid.height; j++) {
            for (int i = 0; i < _materialGrid.width; i++) {
                if (_isCellFluid(i, j, k)) {
                    _fluidCellIndices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }
}

void FluidSimulation::_initializeMarkerParticlesFromSaveState(
                                        FluidSimulationSaveState &state) {
    std::vector<glm::vec3> positions = state.getMarkerParticlePositions();

    _markerParticles.reserve(positions.size());
    glm::vec3 p;
    GridIndex g;
    for (unsigned int i = 0; i < positions.size(); i++) {
        p = positions[i];
        _markerParticles.push_back(MarkerParticle(p));
    }
    positions.clear();
    positions.shrink_to_fit();

    std::vector<glm::vec3> velocities = state.getMarkerParticleVelocities();
    glm::vec3 v;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        v = velocities[i];
        _markerParticles[i].velocity = v;
    }
}

void FluidSimulation::_initializeSolidCellsFromSaveState(FluidSimulationSaveState &state) {
    std::vector<GridIndex> indices = state.getSolidCellIndices();
    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        addSolidCell(g.i, g.j, g.k);
    }
}

void FluidSimulation::_initializeSimulationFromSaveState(FluidSimulationSaveState &state) {
    state.getGridDimensions(&_isize, &_jsize, &_ksize);
    _dx = state.getCellSize();
    _currentFrame = state.getCurrentFrame();
    _currentBrickMeshFrame = _currentFrame;

    _bodyForce = glm::vec3(0.0, 0.0, 0.0);
    _MACVelocity = MACVelocityField(_isize, _jsize, _ksize, _dx);
    _materialGrid = Array3d<int>(_isize, _jsize, _ksize, M_AIR);
    _levelset = LevelSet(_isize, _jsize, _ksize, _dx);
    _markerParticleRadius = pow(3*(_dx*_dx*_dx / 8.0) / (4*3.141592653), 1.0/3.0);

    _initializeSolidCellsFromSaveState(state);
    _initializeMarkerParticlesFromSaveState(state);
    _initializeFluidMaterialParticlesFromSaveState();

    _isFluidInSimulation = _fluidCellIndices.size() > 0;
    _isSimulationInitialized = true;
}

/********************************************************************************
    UPDATE FLUID CELLS
********************************************************************************/

int FluidSimulation::_getUniqueFluidSourceID() {
    int id = _uniqueFluidSourceID;
    _uniqueFluidSourceID++;
    return id;
}

void FluidSimulation::_removeMarkerParticlesFromCells(std::vector<GridIndex> &cells) {
    std::vector<MarkerParticle> newv;
    newv.reserve(_markerParticles.size());

    MarkerParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);
        if (!_isIndexInList(g, cells)) {
            newv.push_back(p);
        }
    }

    _markerParticles = newv;
}

void FluidSimulation::_addNewFluidCells(std::vector<GridIndex> &cells, 
                                        glm::vec3 velocity) {
    _markerParticles.reserve(_markerParticles.size() + 8*cells.size());
    GridIndex g;
    for (unsigned int i = 0; i < cells.size(); i++) {
        _addMarkerParticlesToCell(cells[i], velocity);
    }
}

void FluidSimulation::_updateFluidSource(FluidSource *source) {
    if (source->getSourceType() == T_INFLOW) {
        std::vector<GridIndex> newCells = source->getNewFluidCells(_materialGrid, _dx);
        glm::vec3 velocity = source->getVelocity();

        if (newCells.size() > 0) {
            _addNewFluidCells(newCells, velocity);
        }
    } else if (source->getSourceType() == T_OUTFLOW) {
        std::vector<GridIndex> cells = source->getFluidCells(_materialGrid, _dx);

        if (cells.size() > 0) {
            _removeMarkerParticlesFromCells(cells);
        }
    }
}

void FluidSimulation::_updateFluidSources() {
    for (unsigned int i = 0; i < _fluidSources.size(); i++) {
        _updateFluidSource(_fluidSources[i]);
    }
}

void FluidSimulation::_updateFluidCells() {
    _updateFluidSources();

    _materialGrid.set(_fluidCellIndices, M_AIR);
    _fluidCellIndices.clear();
    
    MarkerParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);
        assert(!_isCellSolid(g));
        _materialGrid.set(g, M_FLUID);
    }

    for (int k = 0; k < _materialGrid.depth; k++) {
        for (int j = 0; j < _materialGrid.height; j++) {
            for (int i = 0; i < _materialGrid.width; i++) {
                if (_isCellFluid(i, j, k)) {
                    _fluidCellIndices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }
}

/********************************************************************************
    FLUID SURFACE RECONSTRUCTION
********************************************************************************/

TriangleMesh FluidSimulation::_polygonizeSurface() {
    ImplicitSurfaceScalarField field = ImplicitSurfaceScalarField(_isize + 1, 
                                                                  _jsize + 1, 
                                                                  _ksize + 1, _dx);
    field.setMaterialGrid(_materialGrid);

    double r = _markerParticleRadius*_markerParticleScale;
    field.setPointRadius(r);

    glm::vec3 p;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i].position;
        field.addPoint(p);
    }

    Polygonizer3d polygonizer = Polygonizer3d(field);
    polygonizer.setInsideCellIndices(_fluidCellIndices);

    polygonizer.polygonizeSurface();
    return polygonizer.getTriangleMesh();
}

void FluidSimulation::_reconstructFluidSurface() {
    _surfaceMesh = _polygonizeSurface();
}

/********************************************************************************
    UPDATE LEVEL SET
********************************************************************************/

void FluidSimulation::_updateLevelSetSignedDistance() {
    _levelset.setSurfaceMesh(_surfaceMesh);

    // Velocities are extrapolated to (_CFLConditionNumber + 2) layers.
    // In order find velocities at the fluid surface for all extrapolated
    // velocity layers, the level set will need to calculate signed distance 
    // for (_CFLConditionNumber + 3) layers
    _levelset.calculateSignedDistanceField((int)ceil(_CFLConditionNumber) + 3);
}

/********************************************************************************
    RECONSTRUCT OUTPUT FLUID SURFACE
********************************************************************************/

void FluidSimulation::_writeDiffuseMaterialToFile(std::string bubblefile,
                                                  std::string foamfile,
                                                  std::string sprayfile) {
    TriangleMesh bubbleMesh;
    TriangleMesh foamMesh;
    TriangleMesh sprayMesh;
    DiffuseParticle dp;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        if (dp.type == DP_BUBBLE) {
            bubbleMesh.vertices.push_back(dp.position);
        } else if (dp.type = DP_FOAM) {
            foamMesh.vertices.push_back(dp.position);
        } else if (dp.type = DP_SPRAY) {
            sprayMesh.vertices.push_back(dp.position);
        }
    }
    bubbleMesh.writeMeshToPLY(bubblefile);
    foamMesh.writeMeshToPLY(foamfile);
    sprayMesh.writeMeshToPLY(sprayfile);
}

void FluidSimulation::_writeSmoothTriangleListToFile(TriangleMesh &mesh, 
                                                     std::string filename) {
    int binsize = mesh.triangles.size();
    char *storage = new char[binsize];

    if (_isSurfaceTriangleSmooth.size() != binsize) {
        _isSurfaceTriangleSmooth = std::vector<bool>();
        _isSurfaceTriangleSmooth.assign(binsize, false);
    }

    bool isSmooth;
    for (int i = 0; i < binsize; i++) {
        isSmooth = _isSurfaceTriangleSmooth[i];
        if (isSmooth) {
            storage[i] = 0x01;
        } else {
            storage[i] = 0x00;
        }
    }
    
    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    file.write(storage, binsize);
    file.close();

    delete[] storage;
}

void FluidSimulation::_writeBrickColorListToFile(TriangleMesh &mesh, 
                                                     std::string filename) {
    int binsize = 3*sizeof(int)*mesh.vertexcolors.size();
    char *storage = new char[binsize];

    int *colordata = new int[3*mesh.vertexcolors.size()];
    glm::vec3 c;
    for (unsigned int i = 0; i < mesh.vertexcolors.size(); i++) {
        c = mesh.vertexcolors[i];
        colordata[3*i] = (int)(c.x*255.0);
        colordata[3*i + 1] = (int)(c.y*255.0);
        colordata[3*i + 2] = (int)(c.z*255.0);
    }
    memcpy(storage, colordata, 3*sizeof(int)*mesh.vertexcolors.size());
    delete[] colordata;
    
    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    file.write(storage, binsize);
    file.close();

    delete[] storage;
}

void FluidSimulation::_writeBrickMaterialToFile(std::string brickfile,
                                                std::string colorfile) {
    TriangleMesh brickmesh;
    _fluidBrickGrid.getBrickMesh(_levelset, brickmesh);

    brickmesh.writeMeshToPLY(brickfile);
    _writeBrickColorListToFile(brickmesh, colorfile);
}

void FluidSimulation::_writeSurfaceMeshToFile(TriangleMesh &mesh) {
    std::string currentFrame = std::to_string(_currentFrame);
    currentFrame.insert(currentFrame.begin(), 6 - currentFrame.size(), '0');

    if (_isSurfaceMeshOutputEnabled) {
        mesh.writeMeshToPLY("bakefiles/" + currentFrame + ".ply");
        _writeSmoothTriangleListToFile(mesh, "bakefiles/smoothfacelist" + currentFrame + ".data");
    }

    if (_isDiffuseMaterialOutputEnabled) {
        _writeDiffuseMaterialToFile("bakefiles/bubble" + currentFrame + ".ply",
                                    "bakefiles/foam" + currentFrame + ".ply",
                                    "bakefiles/spray" + currentFrame + ".ply");
    }

    if (_isBrickOutputEnabled && _fluidBrickGrid.isBrickMeshReady()) {
        std::string currentBrickMeshFrame = std::to_string(_currentBrickMeshFrame);
        currentBrickMeshFrame.insert(currentBrickMeshFrame.begin(), 6 - currentBrickMeshFrame.size(), '0');

        _writeBrickMaterialToFile("bakefiles/brick" + currentBrickMeshFrame + ".ply", 
                                  "bakefiles/brickcolor" + currentBrickMeshFrame + ".data");
        _currentBrickMeshFrame++;
    }
}

bool FluidSimulation::_isVertexNearSolid(glm::vec3 v, double eps) {
    GridIndex g = Grid3d::positionToGridIndex(v, _dx);
    if (_isCellSolid(g)) {
        return true;
    }
    
    // is v near the solid boundary?
    glm::vec3 e = glm::vec3(eps, eps, eps);
    if (g.i == 1 || g.i == _isize - 2 || 
        g.j == 1 || g.j == _jsize - 2 || 
        g.k == 1 || g.k == _ksize - 2) {

        glm::vec3 min = Grid3d::GridIndexToPosition(1, 1, 1, _dx);
        glm::vec3 max = Grid3d::GridIndexToPosition(_isize - 2, 
                                                    _jsize - 2, 
                                                    _ksize - 2, _dx);
        AABB bbox = AABB(min + e, max + glm::vec3(_dx, _dx, _dx) - e);
        if (!bbox.isPointInside(v)) {
            return true;
        }
    }

    // is v near a solid cell?
    glm::vec3 gp = Grid3d::GridIndexToPosition(g, _dx);
    AABB bbox = AABB(gp + e, gp + glm::vec3(_dx, _dx, _dx) - e);
    if (bbox.isPointInside(v)) {
        return false;
    }

    glm::vec3 points[26] {
        v-e.x, v+e.x, v-e.y, v+e.y, v-e.z, v+e.z, v-e.x-e.y, v-e.x+e.y,
        v+e.x-e.y, v+e.x+e.y, v-e.x-e.z, v-e.x+e.z, v+e.x-e.z, v+e.x+e.z, 
        v-e.y-e.z, v-e.y+e.z, v+e.y-e.z, v+e.y+e.z, v-e.x-e.y-e.z,
        v-e.x-e.y+e.z, v-e.x+e.y-e.z, v-e.x+e.y+e.z, v+e.x-e.y-e.z,
        v+e.x-e.y+e.z, v+e.x+e.y-e.z, v+e.x+e.y+e.z};
    
    int i, j, k;
    for (int idx = 0; idx < 26; idx++) {
        Grid3d::positionToGridIndex(points[idx], _dx, &i, &j, &k);
        if (_isCellSolid(i, j, k)) {
            return true;
        }
    }

    return false;
}

void FluidSimulation::_getSmoothVertices(TriangleMesh &mesh,
                                         std::vector<int> &smoothVertices) {
    double eps = 0.02*_dx;
    glm::vec3 v;
    for (unsigned int i = 0; i < mesh.vertices.size(); i++) {
        v = mesh.vertices[i];
        if (!_isVertexNearSolid(v, eps)) {
            smoothVertices.push_back(i);
        }
    }
}

void FluidSimulation::_updateSmoothTriangleList(TriangleMesh &mesh, 
                                                std::vector<int> &smoothVertices) {
    int vsize = mesh.vertices.size();
    int tsize = mesh.triangles.size();

    std::vector<bool> isSmooth;
    isSmooth.assign(vsize, false);
    for (unsigned int i = 0; i < smoothVertices.size(); i++) {
        assert(smoothVertices[i] >= 0 && smoothVertices[i] < vsize);
        isSmooth[smoothVertices[i]] = true;
    }

    std::vector<bool> isTriangleSmooth;
    isTriangleSmooth.assign(tsize, false);
    Triangle t;
    for (int i = 0; i < tsize; i++) {
        t = mesh.triangles[i];
        if (isSmooth[t.tri[0]] || isSmooth[t.tri[1]] || isSmooth[t.tri[2]]) {
            isTriangleSmooth[i] = true;
        }
    }

    _isSurfaceTriangleSmooth = isTriangleSmooth;
}

void FluidSimulation::_smoothSurfaceMesh(TriangleMesh &mesh) {
    std::vector<int> smoothVertices;
    _getSmoothVertices(mesh, smoothVertices);

    mesh.smooth(_surfaceReconstructionSmoothingValue, 
                _surfaceReconstructionSmoothingIterations,
                smoothVertices);
    _updateSmoothTriangleList(mesh, smoothVertices);
}

void FluidSimulation::_getSubdividedSurfaceCells(std::vector<GridIndex> &cells) {
    int subd = _outputFluidSurfaceSubdivisionLevel;
    int numSubdivisions = subd*subd*subd;
    GridIndex *subdivisions = new GridIndex[numSubdivisions];

    double width = _outputFluidSurfaceCellNarrowBandSize*_dx;
    GridIndex g;
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                g = GridIndex(i, j, k);
                if (_levelset.getDistance(g) <= width) {
                    Grid3d::getSubdividedGridIndices(g, subd, subdivisions);
                    for (int idx = 0; idx < numSubdivisions; idx++) {
                        cells.push_back(subdivisions[idx]);
                    }
                }
            }
        }
    }

    delete[] subdivisions;
}

void FluidSimulation::_getSubdividedSolidCells(std::vector<GridIndex> &cells) {
    int subd = _outputFluidSurfaceSubdivisionLevel;
    int numSubdivisions = subd*subd*subd;
    GridIndex *subdivisions = new GridIndex[numSubdivisions];

    GridIndex g;
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                g = GridIndex(i, j, k);
                if (_isCellSolid(g)) {
                    Grid3d::getSubdividedGridIndices(g, subd, subdivisions);
                    for (int idx = 0; idx < numSubdivisions; idx++) {
                        cells.push_back(subdivisions[idx]);
                    }
                }
            }
        }
    }

    delete[] subdivisions;
}

void FluidSimulation::_getOutputSurfaceParticles(std::vector<glm::vec3> &particles) {
    double width = _outputFluidSurfaceParticleNarrowBandSize*_dx;
    glm::vec3 p;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i].position;
        if (_levelset.getDistance(p) <= width) {
            particles.push_back(p);
        }
    }
}

TriangleMesh FluidSimulation::_polygonizeOutputSurface() {

    std::vector<GridIndex> surfaceCells;
    std::vector<GridIndex> solidCells;
    _getSubdividedSurfaceCells(surfaceCells);
    _getSubdividedSolidCells(solidCells);

    std::vector<glm::vec3> particles;
    _getOutputSurfaceParticles(particles);

    int subd = _outputFluidSurfaceSubdivisionLevel;
    int width = _isize*subd;
    int height = _jsize*subd;
    int depth = _ksize*subd;
    double dx = _dx / subd;

    SparseImplicitSurfaceScalarField field = SparseImplicitSurfaceScalarField(width + 1, 
                                                                              height + 1, 
                                                                              depth + 1, dx);
    field.setSolidCells(solidCells);

    double r = _markerParticleRadius*_markerParticleScale;
    field.setPointRadius(r);

    glm::vec3 p;
    for (unsigned int i = 0; i < particles.size(); i++) {
        field.addPoint(particles[i]);
    }

    SparsePolygonizer3d polygonizer = SparsePolygonizer3d(field);
    polygonizer.setSurfaceCellIndices(surfaceCells);
    polygonizer.polygonizeSurface();

    return polygonizer.getTriangleMesh();
}

void FluidSimulation::_updateBrickGrid(double dt) {
    std::vector<glm::vec3> points;
    points.reserve(_markerParticles.size());
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        points.push_back(_markerParticles[i].position);
    }

    _fluidBrickGrid.update(_levelset, points, dt);
}

void FluidSimulation::_reconstructOutputFluidSurface(double dt) {
    
    TriangleMesh mesh;
    if (_isSurfaceMeshOutputEnabled) {
        if (_outputFluidSurfaceSubdivisionLevel == 1) {
            mesh = _surfaceMesh;
        } else {
            mesh = _polygonizeOutputSurface();
        }
        _smoothSurfaceMesh(mesh);
    }

    if (_isBrickOutputEnabled) {
        _updateBrickGrid(dt);
    }

    _writeSurfaceMeshToFile(mesh);
}

/********************************************************************************
    ADVECT FLUID VELOCITIES
********************************************************************************/

void FluidSimulation::_computeVelocityScalarField(Array3d<float> &field,
                                                  Array3d<float> &weightfield, 
                                                  int dir) {
    int U = 0; int V = 1; int W = 2;

    ImplicitSurfaceScalarField grid = ImplicitSurfaceScalarField(field.width,
                                                                 field.height,
                                                                 field.depth, _dx);
    //grid.setTrilinearWeighting();
    grid.enableWeightField();
    grid.setPointRadius(_dx); 

    glm::vec3 offset;
    if (dir == U) {
        offset = glm::vec3(0.0, 0.5*_dx, 0.5*_dx);
    } else if (dir == V) {
        offset = glm::vec3(0.5*_dx, 0.0, 0.5*_dx);
    } else if (dir == W) {
        offset = glm::vec3(0.5*_dx, 0.5*_dx, 0.0);
    } else {
        return;
    }

    MarkerParticle p;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        grid.addPointValue(p.position - offset, p.velocity[dir]);
    }
    grid.applyWeightField();

    grid.getScalarField(field);
    grid.getWeightField(weightfield);
}

void FluidSimulation::_advectVelocityFieldU() {
    _MACVelocity.clearU();

    Array3d<float> ugrid = Array3d<float>(_isize + 1, _jsize, _ksize, 0.0f);
    Array3d<float> weightfield = Array3d<float>(_isize + 1, _jsize, _ksize, 0.0f);
    _computeVelocityScalarField(ugrid, weightfield, 0);

    std::vector<GridIndex> extrapolationIndices;
    double eps = 10e-9;
    for (int k = 0; k < ugrid.depth; k++) {
        for (int j = 0; j < ugrid.height; j++) {
            for (int i = 0; i < ugrid.width; i++) {
                if (_isFaceBorderingMaterialU(i, j, k, M_FLUID)) {
                    if (weightfield(i, j, k) < eps) {
                        extrapolationIndices.push_back(GridIndex(i, j, k));
                    } else {
                        _MACVelocity.setU(i, j, k, ugrid(i, j, k));
                    }
                }
            }
        }
    }

    GridIndex g, n;
    GridIndex nb[26];
    double avg, weight;
    for (unsigned int i = 0; i < extrapolationIndices.size(); i++) {
        g = extrapolationIndices[i];
        Grid3d::getNeighbourGridIndices26(g, nb);

        avg = 0.0;
        weight = 0.0;
        for (int idx = 0; idx < 26; idx++) {
            n = nb[idx];
            if (ugrid.isIndexInRange(n) && weightfield(n) > 0.0) {
                avg += ugrid(n);
                weight += 1.0;
            }
        }

        if (weight > 0.0) {
            _MACVelocity.setU(g, avg / weight);
        }
    }
}

void FluidSimulation::_advectVelocityFieldV() {
    _MACVelocity.clearV();

    Array3d<float> vgrid = Array3d<float>(_isize, _jsize + 1, _ksize, 0.0f);
    Array3d<float> weightfield = Array3d<float>(_isize, _jsize + 1, _ksize, 0.0f);
    _computeVelocityScalarField(vgrid, weightfield, 1);
    
    std::vector<GridIndex> extrapolationIndices;
    double eps = 10e-9;
    for (int k = 0; k < vgrid.depth; k++) {
        for (int j = 0; j < vgrid.height; j++) {
            for (int i = 0; i < vgrid.width; i++) {
                if (_isFaceBorderingMaterialV(i, j, k, M_FLUID)) {
                    if (weightfield(i, j, k) < eps) {
                        extrapolationIndices.push_back(GridIndex(i, j, k));
                    } else {
                        _MACVelocity.setV(i, j, k, vgrid(i, j, k));
                    }
                }
            }
        }
    }

    GridIndex g, n;
    GridIndex nb[26];
    double avg, weight;
    for (unsigned int i = 0; i < extrapolationIndices.size(); i++) {
        g = extrapolationIndices[i];
        Grid3d::getNeighbourGridIndices26(g, nb);

        avg = 0.0;
        weight = 0.0;
        for (int idx = 0; idx < 26; idx++) {
            n = nb[idx];
            if (vgrid.isIndexInRange(n) && weightfield(n) > 0.0) {
                avg += vgrid(n);
                weight += 1.0;
            }
        }

        if (weight > 0.0) {
            _MACVelocity.setV(g, avg / weight);
        }
    }
}

void FluidSimulation::_advectVelocityFieldW() {
    _MACVelocity.clearW();

    Array3d<float> wgrid = Array3d<float>(_isize, _jsize, _ksize + 1, 0.0f);
    Array3d<float> weightfield = Array3d<float>(_isize, _jsize, _ksize + 1, 0.0f);
    _computeVelocityScalarField(wgrid, weightfield, 2);
    
    std::vector<GridIndex> extrapolationIndices;
    double eps = 10e-9;
    for (int k = 0; k < wgrid.depth; k++) {
        for (int j = 0; j < wgrid.height; j++) {
            for (int i = 0; i < wgrid.width; i++) {
                if (_isFaceBorderingMaterialW(i, j, k, M_FLUID)) {
                    if (weightfield(i, j, k) < eps) {
                        extrapolationIndices.push_back(GridIndex(i, j, k));
                    } else {
                        _MACVelocity.setW(i, j, k, wgrid(i, j, k));
                    }
                }
            }
        }
    }

    GridIndex g, n;
    GridIndex nb[26];
    double avg, weight;
    for (unsigned int i = 0; i < extrapolationIndices.size(); i++) {
        g = extrapolationIndices[i];
        Grid3d::getNeighbourGridIndices26(g, nb);

        avg = 0.0;
        weight = 0.0;
        for (int idx = 0; idx < 26; idx++) {
            n = nb[idx];
            if (wgrid.isIndexInRange(n) && weightfield(n) > 0.0) {
                avg += wgrid(n);
                weight += 1.0;
            }
        }

        if (weight > 0.0) {
            _MACVelocity.setW(g, avg / weight);
        }
    }
}

void FluidSimulation::_advectVelocityField() {
    _advectVelocityFieldU();
    _advectVelocityFieldV();
    _advectVelocityFieldW();
}

/********************************************************************************
    APPLY BODY FORCES
********************************************************************************/

void FluidSimulation::_applyBodyForcesToVelocityField(double dt) {
    if (fabs(_bodyForce.x) > 0.0) {
        for (int k = 0; k < _ksize; k++) {
            for (int j = 0; j < _jsize; j++) {
                for (int i = 0; i < _isize + 1; i++) {
                    if (_isFaceBorderingMaterialU(i, j, k, M_FLUID)) {
                        _MACVelocity.addU(i, j, k, _bodyForce.x * dt);
                    }
                }
            }
        }
    }

    if (fabs(_bodyForce.y) > 0.0) {
        for (int k = 0; k < _ksize; k++) {
            for (int j = 0; j < _jsize + 1; j++) {
                for (int i = 0; i < _isize; i++) {
                    if (_isFaceBorderingMaterialV(i, j, k, M_FLUID)) {
                        _MACVelocity.addV(i, j, k, _bodyForce.y * dt);
                    }
                }
            }
        }
    }

    if (fabs(_bodyForce.z) > 0.0) {
        for (int k = 0; k < _ksize + 1; k++) {
            for (int j = 0; j < _jsize; j++) {
                for (int i = 0; i < _isize; i++) {
                    if (_isFaceBorderingMaterialW(i, j, k, M_FLUID)) {
                        _MACVelocity.addW(i, j, k, _bodyForce.z * dt);
                    }
                }
            }
        }
    }
}

glm::vec3 FluidSimulation::_getVelocityAtPosition(glm::vec3 p) {
    return _MACVelocity.evaluateVelocityAtPosition(p);
}


/********************************************************************************
    UPDATE PRESSURE GRID
********************************************************************************/

double FluidSimulation::_calculateNegativeDivergenceVector(VectorCoefficients &b) {
    double scale = 1.0f / (float)_dx;

    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        int i = _fluidCellIndices[idx].i;
        int j = _fluidCellIndices[idx].j;
        int k = _fluidCellIndices[idx].k;

        double value = -scale * (double)(_MACVelocity.U(i + 1, j, k) - _MACVelocity.U(i, j, k) +
                                         _MACVelocity.V(i, j + 1, k) - _MACVelocity.V(i, j, k) +
                                         _MACVelocity.W(i, j, k + 1) - _MACVelocity.W(i, j, k));
        b.vector.set(i, j, k, (float)value);
    }

    // solid cells are stationary right now
    float usolid = 0.0;
    float vsolid = 0.0;
    float wsolid = 0.0;
    float maxDivergence = 0.0;
    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        int i = _fluidCellIndices[idx].i;
        int j = _fluidCellIndices[idx].j;
        int k = _fluidCellIndices[idx].k;

        if (_isCellSolid(i-1, j, k)) {
            float value = b.vector(i, j, k) - (float)scale*(_MACVelocity.U(i, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }
        if (_isCellSolid(i+1, j, k)) {
            float value = b.vector(i, j, k) + (float)scale*(_MACVelocity.U(i+1, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }

        if (_isCellSolid(i, j-1, k)) {
            float value = b.vector(i, j, k) - (float)scale*(_MACVelocity.V(i, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }
        if (_isCellSolid(i, j+1, k)) {
            float value = b.vector(i, j, k) + (float)scale*(_MACVelocity.V(i, j+1, k) - usolid);
            b.vector.set(i, j, k, value);
        }

        if (_isCellSolid(i, j, k-1)) {
            float value = b.vector(i, j, k) - (float)scale*(_MACVelocity.W(i, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }
        if (_isCellSolid(i, j, k+1)) {
            float value = b.vector(i, j, k) + (float)scale*(_MACVelocity.W(i, j, k+1) - usolid);
            b.vector.set(i, j, k, value);
        }

        maxDivergence = fmax(maxDivergence, fabs(b.vector(i, j, k)));
    }

    return (double)maxDivergence;
}

void FluidSimulation::_calculateMatrixCoefficients(MatrixCoefficients &A, double dt) {
    float scale = (float)dt / (float)(_density*_dx*_dx);

    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx ++) {
        int i = _fluidCellIndices[idx].i;
        int j = _fluidCellIndices[idx].j;
        int k = _fluidCellIndices[idx].k;

        if (_isCellFluid(i + 1, j, k)) {
            A.diag.add(i, j, k, scale);
            A.diag.add(i + 1, j, k, scale);
            A.plusi.set(i, j, k, -scale);
        }
        else if (_isCellAir(i + 1, j, k)) {
            A.diag.add(i, j, k, scale);
        }

        if (_isCellFluid(i, j + 1, k)) {
            A.diag.add(i, j, k, scale);
            A.diag.add(i, j + 1, k, scale);
            A.plusj.set(i, j, k, -scale);
        }
        else if (_isCellAir(i, j + 1, k)) {
            A.diag.add(i, j, k, scale);
        }

        if (_isCellFluid(i, j, k + 1)) {
            A.diag.add(i, j, k, scale);
            A.diag.add(i, j, k + 1, scale);
            A.plusk.set(i, j, k, -scale);
        }
        else if (_isCellAir(i, j, k + 1)) {
            A.diag.add(i, j, k, scale);
        }

    }

}

void FluidSimulation::_calculatePreconditionerVector(VectorCoefficients &p, 
                                                     MatrixCoefficients &A) {
    float tau = 0.97f;      // Tuning constant
    float sigma = 0.25f;    // safety constant
    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        int i = _fluidCellIndices[idx].i;
        int j = _fluidCellIndices[idx].j;
        int k = _fluidCellIndices[idx].k;

        float v1 = A.plusi(i - 1, j, k)*p.vector(i - 1, j, k);
        float v2 = A.plusj(i, j - 1, k)*p.vector(i, j - 1, k);
        float v3 = A.plusk(i, j, k - 1)*p.vector(i, j, k - 1);
        float v4 = p.vector(i - 1, j, k); v4 = v4*v4;
        float v5 = p.vector(i, j - 1, k); v5 = v5*v5;
        float v6 = p.vector(i, j, k - 1); v6 = v6*v6;

        float e = A.diag(i, j, k) - v1*v1 - v2*v2 - v3*v3 - 
            tau*(A.plusi(i - 1, j, k)*(A.plusj(i - 1, j, k) + A.plusk(i - 1, j, k))*v4 +
                 A.plusj(i, j - 1, k)*(A.plusi(i, j - 1, k) + A.plusk(i, j - 1, k))*v5 +
                 A.plusk(i, j, k - 1)*(A.plusi(i, j, k - 1) + A.plusj(i, j, k - 1))*v6);

        if (e < sigma*A.diag(i, j, k)) {
            e = A.diag(i, j, k);
        }

        if (fabs(e) > 10e-9) {
            p.vector.set(i, j, k, 1.0f / sqrt(e));
        }
    }
}

Eigen::VectorXd FluidSimulation::_VectorCoefficientsToEigenVectorXd(VectorCoefficients &v,
                                                                    std::vector<GridIndex> indices) {
    Eigen::VectorXd ev((int)indices.size());
    for (unsigned int idx = 0; idx < indices.size(); idx++) {
        int i = indices[idx].i;
        int j = indices[idx].j;
        int k = indices[idx].k;

        ev(idx) = (double)v.vector(i, j, k);
    }

    return ev;
}

void FluidSimulation::_EigenVectorXdToVectorCoefficients(Eigen::VectorXd v, 
                                                         VectorCoefficients &vc) {
    for (int idx = 0; idx < v.size(); idx++) {
        GridIndex index = _VectorIndexToGridIndex(idx);
        vc.vector.set(index, (float)v(idx));
    }
}

void FluidSimulation::_updateFluidGridIndexToEigenVectorXdIndexHashTable(Array3d<int> &hashTable) {
    hashTable.fill(-1);

    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        hashTable.set(_fluidCellIndices[idx], idx);
    }
}

GridIndex FluidSimulation::_VectorIndexToGridIndex(int i) {
    return _fluidCellIndices[i];
}

int FluidSimulation::_GridIndexToVectorIndex(int i, int j, int k, Array3d<int> &hashTable) {
    GridIndex g(i, j, k);
    return _GridIndexToVectorIndex(g, hashTable);
}

int FluidSimulation::_GridIndexToVectorIndex(GridIndex index, Array3d<int> &hashTable) {
    int xdidx = hashTable(index);
    assert(xdidx != -1);
    return xdidx;
}

Eigen::VectorXd FluidSimulation::_applyPreconditioner(Eigen::VectorXd residualVector,
                                                      VectorCoefficients &p,
                                                      MatrixCoefficients &A) {
    VectorCoefficients r(_isize, _jsize, _ksize);
    _EigenVectorXdToVectorCoefficients(residualVector, r);

    // Solve Aq = r
    VectorCoefficients q(_isize, _jsize, _ksize);
    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        GridIndex g = _fluidCellIndices[idx];
        int i = g.i;
        int j = g.j;
        int k = g.k;

        float t = r.vector(i, j, k) -
            A.plusi(i - 1, j, k)*p.vector(i - 1, j, k)*q.vector(i - 1, j, k) -
            A.plusj(i, j - 1, k)*p.vector(i, j - 1, k)*q.vector(i, j - 1, k) -
            A.plusk(i, j, k - 1)*p.vector(i, j, k - 1)*q.vector(i, j, k - 1);

        t = t*p.vector(i, j, k);
        q.vector.set(i, j, k, t);
    }

    // Solve transpose(A)*z = q
    VectorCoefficients z(_isize, _jsize, _ksize);
    for (int idx = (int)_fluidCellIndices.size() - 1; idx >= 0; idx--) {
        GridIndex g = _fluidCellIndices[idx];
        int i = g.i;
        int j = g.j;
        int k = g.k;

        float precon = p.vector(i, j, k);
        float t = q.vector(i, j, k) -
            A.plusi(i, j, k)*precon*z.vector(i + 1, j, k) -
            A.plusj(i, j, k)*precon*z.vector(i, j + 1, k) -
            A.plusk(i, j, k)*precon*z.vector(i, j, k + 1);

        t = t*precon;
        z.vector.set(i, j, k, t);
    }

    return _VectorCoefficientsToEigenVectorXd(z, _fluidCellIndices);
}

int FluidSimulation::_getNumFluidOrAirCellNeighbours(int i, int j, int k) {
    int n = 0;
    if (!_isCellSolid(i-1, j, k)) { n++; }
    if (!_isCellSolid(i+1, j, k)) { n++; }
    if (!_isCellSolid(i, j-1, k)) { n++; }
    if (!_isCellSolid(i, j+1, k)) { n++; }
    if (!_isCellSolid(i, j, k-1)) { n++; }
    if (!_isCellSolid(i, j, k+1)) { n++; }

    return n;
}

Eigen::SparseMatrix<double> FluidSimulation::_MatrixCoefficientsToEigenSparseMatrix(
                                                    MatrixCoefficients &A, 
                                                    Array3d<int> &vectorIndexHashTable,
                                                    double dt) {
    int size = (int)_fluidCellIndices.size();
    double scale = dt / (_density * _dx*_dx);

    std::vector<Eigen::Triplet<double>> matrixValues;
    matrixValues.reserve(size * (6 + 1));   // max number of non-zero entries 
                                            // (6 neighbours + diagonal) for each row
    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        GridIndex g = _fluidCellIndices[idx];
        int i = g.i;
        int j = g.j;
        int k = g.k;

        int row = idx;
        int col = idx;
        double diag = _getNumFluidOrAirCellNeighbours(i, j, k)*scale;
        matrixValues.push_back(Eigen::Triplet<double>(col, row, diag));

        if (_isCellFluid(i-1, j, k)) {
            col = _GridIndexToVectorIndex(i-1, j, k, vectorIndexHashTable);
            double coef = (double)A.plusi(i-1, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
        if (_isCellFluid(i+1, j, k)) {
            col = _GridIndexToVectorIndex(i+1, j, k, vectorIndexHashTable);
            double coef = (double)A.plusi(i, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }

        if (_isCellFluid(i, j-1, k)) {
            col = _GridIndexToVectorIndex(i, j-1, k, vectorIndexHashTable);
            double coef = (double)A.plusj(i, j-1, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
        if (_isCellFluid(i, j+1, k)) {
            col = _GridIndexToVectorIndex(i, j+1, k, vectorIndexHashTable);
            double coef = (double)A.plusj(i, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }

        if (_isCellFluid(i, j, k-1)) {
            col = _GridIndexToVectorIndex(i, j, k-1, vectorIndexHashTable);
            double coef = (double)A.plusk(i, j, k-1);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
        if (_isCellFluid(i, j, k+1)) {
            col = _GridIndexToVectorIndex(i, j, k+1, vectorIndexHashTable);
            double coef = (double)A.plusk(i, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
    }

    Eigen::SparseMatrix<double> m(size, size);
    m.setFromTriplets(matrixValues.begin(), matrixValues.end());

    return m;
}

// Solve (A*p = b) with Modified Incomplete Cholesky Conjugate Gradient menthod
// (MICCG(0))
Eigen::VectorXd FluidSimulation::_solvePressureSystem(MatrixCoefficients &A,
                                                      VectorCoefficients &b,
                                                      VectorCoefficients &precon,
                                                      Array3d<int> &vectorIndexHashTable,
                                                      double dt) {

    int size = (int)_fluidCellIndices.size();
    double tol = _pressureSolveTolerance;

    Eigen::VectorXd pressureVector(size); pressureVector.setZero();
    Eigen::VectorXd bVector = _VectorCoefficientsToEigenVectorXd(b, _fluidCellIndices);
    Eigen::VectorXd residualVector(bVector);

    if (fabs(residualVector.maxCoeff()) < tol) {
        return pressureVector;
    }

    Eigen::SparseMatrix<double> aMatrix = _MatrixCoefficientsToEigenSparseMatrix(A, vectorIndexHashTable, dt);
    Eigen::VectorXd preconVector = _VectorCoefficientsToEigenVectorXd(precon, _fluidCellIndices);
    Eigen::VectorXd auxillaryVector = _applyPreconditioner(residualVector, precon, A);
    Eigen::VectorXd searchVector(auxillaryVector);

    double alpha = 0.0;
    double beta = 0.0;
    double sigma = auxillaryVector.dot(residualVector);
    double sigmaNew = 0.0;
    int iterationNumber = 0;

    while (iterationNumber < _maxPressureSolveIterations) {
        auxillaryVector = aMatrix*searchVector;
        alpha = sigma / auxillaryVector.dot(searchVector);
        pressureVector += alpha*searchVector;
        residualVector -= alpha*auxillaryVector;

        if (fabs(residualVector.maxCoeff()) < tol) {
            _logfile.log("CG Iterations: ", iterationNumber, 1);
            return pressureVector;
        }

        auxillaryVector = _applyPreconditioner(residualVector, precon, A);
        sigmaNew = auxillaryVector.dot(residualVector);
        beta = sigmaNew / sigma;
        searchVector = auxillaryVector + beta*searchVector;
        sigma = sigmaNew;

        iterationNumber++;

        if (iterationNumber % 10 == 0) {
            std::cout << "\tIteration #: " << iterationNumber <<
                         "\tEstimated Error: " << fabs(residualVector.maxCoeff()) << std::endl;
        }
    }

    _logfile.log("Iterations limit reached.\t Estimated error : ",
                 fabs(residualVector.maxCoeff()), 1);

    return pressureVector;
}

void FluidSimulation::_updatePressureGrid(Array3d<float> &pressureGrid, double dt) {

    VectorCoefficients b(_isize, _jsize, _ksize);
    double maxDivergence = _calculateNegativeDivergenceVector(b);
    if (maxDivergence < _pressureSolveTolerance) {
        // all pressure values are near 0.0
        return;
    }

    MatrixCoefficients matrixA = MatrixCoefficients(_isize, _jsize, _ksize);
    Array3d<int> vectorIndexHashTable = Array3d<int>(_isize, _jsize, _ksize, -1);
    VectorCoefficients preconditioner = VectorCoefficients(_isize, _jsize, _ksize);

    _calculateMatrixCoefficients(matrixA, dt);
    _calculatePreconditionerVector(preconditioner, matrixA);
    _updateFluidGridIndexToEigenVectorXdIndexHashTable(vectorIndexHashTable);
    Eigen::VectorXd pressures = _solvePressureSystem(matrixA, b, preconditioner,
                                                     vectorIndexHashTable, dt);
    
    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        GridIndex index = _VectorIndexToGridIndex(idx);
        pressureGrid.set(index, (float)pressures(idx));
    }
}

/********************************************************************************
    APPLY PRESSURE TO VELOCITY FIELD
********************************************************************************/

void FluidSimulation::_applyPressureToFaceU(int i, int j, int k, 
                                            Array3d<float> &pressureGrid,
                                            MACVelocityField &tempMACVelocity, double dt) {
    double usolid = 0.0;   // solids are stationary
    double scale = dt / (_density * _dx);
    double invscale = 1.0 / scale;

    int ci = i - 1; int cj = j; int ck = k;

    double p0, p1;
    if (!_isCellSolid(ci, cj, ck) && !_isCellSolid(ci + 1, cj, ck)) {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci + 1, cj, ck);
    } else if (_isCellSolid(ci, cj, ck)) {
        p0 = pressureGrid(ci + 1, cj, ck) - 
                invscale*(_MACVelocity.U(i, j, k) - usolid);
        p1 = pressureGrid(ci + 1, cj, ck);
    } else {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck) +
                invscale*(_MACVelocity.U(i, j, k) - usolid);
    }

    double unext = _MACVelocity.U(i, j, k) - scale*(p1 - p0);
    tempMACVelocity.setU(i, j, k, unext);
}

void FluidSimulation::_applyPressureToFaceV(int i, int j, int k, 
                                            Array3d<float> &pressureGrid,
                                            MACVelocityField &tempMACVelocity, double dt) {
    double usolid = 0.0;   // solids are stationary
    double scale = dt / (_density * _dx);
    double invscale = 1.0 / scale;

    int ci = i; int cj = j - 1; int ck = k;

    double p0, p1;
    if (!_isCellSolid(ci, cj, ck) && !_isCellSolid(ci, cj + 1, ck)) {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj + 1, ck);
    }
    else if (_isCellSolid(ci, cj, ck)) {
        p0 = pressureGrid(ci, cj + 1, ck) -
            invscale*(_MACVelocity.V(i, j, k) - usolid);
        p1 = pressureGrid(ci, cj + 1, ck);
    }
    else {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck) +
            invscale*(_MACVelocity.V(i, j, k) - usolid);
    }

    double vnext = _MACVelocity.V(i, j, k) - scale*(p1 - p0);
    tempMACVelocity.setV(i, j, k, vnext);
}

void FluidSimulation::_applyPressureToFaceW(int i, int j, int k, 
                                            Array3d<float> &pressureGrid,
                                            MACVelocityField &tempMACVelocity, double dt) {
    double usolid = 0.0;   // solids are stationary
    double scale = dt / (_density * _dx);
    double invscale = 1.0 / scale;

    int ci = i; int cj = j; int ck = k - 1;

    double p0, p1;
    if (!_isCellSolid(ci, cj, ck) && !_isCellSolid(ci, cj, ck + 1)) {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck + 1);
    }
    else if (_isCellSolid(ci, cj, ck)) {
        p0 = pressureGrid(ci, cj, ck + 1) -
                invscale*(_MACVelocity.W(i, j, k) - usolid);
        p1 = pressureGrid(ci, cj, ck + 1);
    }
    else {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck) +
                invscale*(_MACVelocity.W(i, j, k) - usolid);
    }

    double wnext = _MACVelocity.W(i, j, k) - scale*(p1 - p0);
    tempMACVelocity.setW(i, j, k, wnext);
}

void FluidSimulation::_commitTemporaryVelocityFieldValues(MACVelocityField &tempMACVelocity) {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                if (_isFaceBorderingMaterialU(i, j, k, M_FLUID)) {
                    _MACVelocity.setU(i, j, k, tempMACVelocity.U(i, j, k));
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isFaceBorderingMaterialV(i, j, k, M_FLUID)) {
                    _MACVelocity.setV(i, j, k, tempMACVelocity.V(i, j, k));
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isFaceBorderingMaterialW(i, j, k, M_FLUID)) {
                    _MACVelocity.setW(i, j, k, tempMACVelocity.W(i, j, k));
                }
            }
        }
    }
}

void FluidSimulation::_applyPressureToVelocityField(Array3d<float> &pressureGrid, double dt) {
    MACVelocityField tempMACVelocity = MACVelocityField(_isize, _jsize, _ksize, _dx);

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                if (_isFaceBorderingMaterialU(i, j, k, M_SOLID)) {
                    tempMACVelocity.setU(i, j, k, 0.0);
                }

                if (_isFaceBorderingMaterialU(i, j, k, M_FLUID) &&
                    !_isFaceBorderingMaterialU(i, j, k, M_SOLID)) {
                    _applyPressureToFaceU(i, j, k, pressureGrid, tempMACVelocity, dt);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isFaceBorderingMaterialV(i, j, k, M_SOLID)) {
                    tempMACVelocity.setV(i, j, k, 0.0);
                }

                if (_isFaceBorderingMaterialV(i, j, k, M_FLUID) &&
                    !_isFaceBorderingMaterialV(i, j, k, M_SOLID)) {
                    _applyPressureToFaceV(i, j, k, pressureGrid, tempMACVelocity, dt);
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isFaceBorderingMaterialW(i, j, k, M_SOLID)) {
                    tempMACVelocity.setW(i, j, k, 0.0);
                }

                if (_isFaceBorderingMaterialW(i, j, k, M_FLUID) &&
                    !_isFaceBorderingMaterialW(i, j, k, M_SOLID)) {
                    _applyPressureToFaceW(i, j, k, pressureGrid, tempMACVelocity, dt);
                }
            }
        }
    }

    _commitTemporaryVelocityFieldValues(tempMACVelocity);
}

/********************************************************************************
    EXTRAPOLATE FLUID VELOCITIES
********************************************************************************/

void FluidSimulation::_updateExtrapolationLayer(int layerIndex, Array3d<int> &layerGrid) {
    GridIndex neighbours[6];
    GridIndex n;

    for (int k = 0; k < layerGrid.depth; k++) {
        for (int j = 0; j < layerGrid.height; j++) {
            for (int i = 0; i < layerGrid.width; i++) {
                if (layerGrid(i, j, k) == layerIndex - 1 && !_isCellSolid(i, j, k)) {
                    Grid3d::getNeighbourGridIndices6(i, j, k, neighbours);
                    for (int idx = 0; idx < 6; idx++) {
                        n = neighbours[idx];

                        if (Grid3d::isGridIndexInRange(n, _isize, _jsize, _ksize) && 
                                layerGrid(n) == -1 && !_isCellSolid(n)) {
                            layerGrid.set(n, layerIndex);
                        }
                    }
                }
            }
        }
    }
}

int FluidSimulation::_updateExtrapolationLayers(Array3d<int> &layerGrid) {

    GridIndex idx;
    for (unsigned int i = 0; i < _fluidCellIndices.size(); i++) {
        idx = _fluidCellIndices[i];
        layerGrid.set(idx, 0);
    }

    // add 2 extra layers to account for extra values needed during cubic 
    // interpolation calculations
    int numLayers = (int)ceil(_CFLConditionNumber) + 2;
    for (int layer = 1; layer <= numLayers; layer++) {
        _updateExtrapolationLayer(layer, layerGrid);
    }

    return numLayers;
}

double FluidSimulation::_getExtrapolatedVelocityForFaceU(int i, int j, int k, int layerIdx,
                                                         Array3d<int> &layerGrid) {

    // First two layers are calculate by averaging neighbours so that values exist for tricubic
    // interpolation at the fluid boundary for layers > 2
    if (layerIdx > 2) {
        glm::vec3 pos = _MACVelocity.velocityIndexToPositionU(i, j, k);
        glm::vec3 v = _getVelocityAtNearestPointOnFluidSurface(pos);
        return v.x;
    }

    GridIndex n[6];
    Grid3d::getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (_MACVelocity.isIndexInRangeU(c) && _isFaceBorderingLayerIndexU(c, layerIdx - 1, layerGrid)) {
                sum += _MACVelocity.U(c);
                weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

double FluidSimulation::_getExtrapolatedVelocityForFaceV(int i, int j, int k, int layerIdx,
                                                         Array3d<int> &layerGrid) {

    if (layerIdx > 2) {
        glm::vec3 pos = _MACVelocity.velocityIndexToPositionV(i, j, k);
        glm::vec3 v = _getVelocityAtNearestPointOnFluidSurface(pos);
        return v.y;
    }

    GridIndex n[6];
    Grid3d::getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (_MACVelocity.isIndexInRangeV(c) && _isFaceBorderingLayerIndexV(c, layerIdx - 1, layerGrid)) {
            sum += _MACVelocity.V(c);
            weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

double FluidSimulation::_getExtrapolatedVelocityForFaceW(int i, int j, int k, int layerIdx,
                                                         Array3d<int> &layerGrid) {

    if (layerIdx > 2) {
        glm::vec3 pos = _MACVelocity.velocityIndexToPositionW(i, j, k);
        glm::vec3 v = _getVelocityAtNearestPointOnFluidSurface(pos);
        return v.z;
    }

    GridIndex n[6];
    Grid3d::getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (_MACVelocity.isIndexInRangeW(c) && _isFaceBorderingLayerIndexW(c, layerIdx - 1, layerGrid)) {
            sum += _MACVelocity.W(c);
            weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

glm::vec3 FluidSimulation::_getVelocityAtNearestPointOnFluidSurface(glm::vec3 p) {
    p = _levelset.getClosestPointOnSurface(p);
    return _getVelocityAtPosition(p);
}

void FluidSimulation::_extrapolateVelocitiesForLayerIndexU(int idx, Array3d<int> &layerGrid) {
    Array3d<float> tempMACVelocityU = Array3d<float>(_isize + 1, _jsize, _ksize);
    std::vector<GridIndex> tempIndices;

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                bool isExtrapolated = _isFaceBorderingLayerIndexU(i, j, k, idx, layerGrid) && 
                                     !_isFaceBorderingLayerIndexU(i, j, k, idx-1, layerGrid) &&
                                    (!_isFaceBorderingMaterialU(i, j, k, M_SOLID));
                if (isExtrapolated) {
                    double v = _getExtrapolatedVelocityForFaceU(i, j, k, idx, layerGrid);
                    tempMACVelocityU.set(i, j, k, (float)v);
                    tempIndices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

    for (unsigned int i = 0; i < tempIndices.size(); i++) {
        _MACVelocity.setU(tempIndices[i], tempMACVelocityU(tempIndices[i]));
    }
}

void FluidSimulation::_extrapolateVelocitiesForLayerIndexV(int idx, Array3d<int> &layerGrid) {
    Array3d<float> tempMACVelocityV = Array3d<float>(_isize, _jsize + 1, _ksize);
    std::vector<GridIndex> tempIndices;

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                bool isExtrapolated = _isFaceBorderingLayerIndexV(i, j, k, idx, layerGrid) && 
                                     !_isFaceBorderingLayerIndexV(i, j, k, idx-1, layerGrid) &&
                                    (!_isFaceBorderingMaterialV(i, j, k, M_SOLID));
                if (isExtrapolated) {
                    double v = _getExtrapolatedVelocityForFaceV(i, j, k, idx, layerGrid);
                    tempMACVelocityV.set(i, j, k, (float)v);
                    tempIndices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

    for (unsigned int i = 0; i < tempIndices.size(); i++) {
        _MACVelocity.setV(tempIndices[i], tempMACVelocityV(tempIndices[i]));
    }
}

void FluidSimulation::_extrapolateVelocitiesForLayerIndexW(int idx, Array3d<int> &layerGrid) {
    Array3d<float> tempMACVelocityW = Array3d<float>(_isize, _jsize, _ksize + 1);
    std::vector<GridIndex> tempIndices;

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                bool isExtrapolated = _isFaceBorderingLayerIndexW(i, j, k, idx, layerGrid) && 
                                     !_isFaceBorderingLayerIndexW(i, j, k, idx-1, layerGrid) &&
                                    (!_isFaceBorderingMaterialW(i, j, k, M_SOLID));
                if (isExtrapolated) {
                    double v = _getExtrapolatedVelocityForFaceW(i, j, k, idx, layerGrid);
                    tempMACVelocityW.set(i, j, k, (float)v);
                    tempIndices.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

    for (unsigned int i = 0; i < tempIndices.size(); i++) {
        _MACVelocity.setW(tempIndices[i], tempMACVelocityW(tempIndices[i]));
    }
}

void FluidSimulation::_extrapolateVelocitiesForLayerIndex(int idx, Array3d<int> &layerGrid) {
    _extrapolateVelocitiesForLayerIndexU(idx, layerGrid);
    _extrapolateVelocitiesForLayerIndexV(idx, layerGrid);
    _extrapolateVelocitiesForLayerIndexW(idx, layerGrid);
}

void FluidSimulation::_resetExtrapolatedFluidVelocities() {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                if (!_isFaceBorderingMaterialU(i, j, k, M_FLUID)) {
                    _MACVelocity.setU(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                if (!_isFaceBorderingMaterialV(i, j, k, M_FLUID)) {
                    _MACVelocity.setV(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (!_isFaceBorderingMaterialW(i, j, k, M_FLUID)) {
                    _MACVelocity.setW(i, j, k, 0.0);
                }
            }
        }
    }
}

void FluidSimulation::_extrapolateFluidVelocities() {
    Array3d<int> layerGrid = Array3d<int>(_isize, _jsize, _ksize, -1);

    _resetExtrapolatedFluidVelocities();
    int numLayers = _updateExtrapolationLayers(layerGrid);

    for (int i = 1; i <= numLayers; i++) {
        _extrapolateVelocitiesForLayerIndex(i, layerGrid);
    }
}


/********************************************************************************
    UPDATE DIFFUSE MATERIAL PARTICLES
********************************************************************************/

void FluidSimulation::_sortMarkerParticlePositions(std::vector<glm::vec3> &surface, 
                                                   std::vector<glm::vec3> &inside) {
    glm::vec3 p;
    double width = _diffuseSurfaceNarrowBandSize * _dx;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i].position;
        if (_levelset.getDistance(p) < width) {
            surface.push_back(p);
        } else if (_levelset.isPointInInsideCell(p)) {
            inside.push_back(p);
        }
    }
}

double FluidSimulation::_getWavecrestPotential(glm::vec3 p, glm::vec3 *v) {

    *v = _getVelocityAtPosition(p);
    glm::vec3 normal;
    double k = _levelset.getSurfaceCurvature(p, &normal);

    if (glm::dot(glm::normalize(*v), normal) < 0.6) {
        return 0.0;
    }

    k = fmax(k, _minWavecrestCurvature);
    k = fmin(k, _maxWavecrestCurvature);

    return (k - _minWavecrestCurvature) / 
           (_maxWavecrestCurvature - _minWavecrestCurvature);
}

double FluidSimulation::_getTurbulencePotential(glm::vec3 p, TurbulenceField &tfield) {
    double t = tfield.evaluateTurbulenceAtPosition(p);

    t = fmax(t, _minTurbulence);
    t = fmin(t, _maxTurbulence);

    return (t - _minTurbulence) / 
           (_maxTurbulence - _minTurbulence);
}

double FluidSimulation::_getEnergyPotential(glm::vec3 velocity) {
    double e = 0.5*glm::dot(velocity, velocity);
    e = fmax(e, _minParticleEnergy);
    e = fmin(e, _maxParticleEnergy);

    return (e - _minParticleEnergy) / (_maxParticleEnergy - _minParticleEnergy);
}

void FluidSimulation::_getSurfaceDiffuseParticleEmitters(
                            std::vector<glm::vec3> &surface, 
                            std::vector<DiffuseParticleEmitter> &emitters) {
    glm::vec3 p;
    for (unsigned int i = 0; i < surface.size(); i++) {
        p = surface[i];

        glm::vec3 velocity;
        double Iwc = _getWavecrestPotential(p, &velocity);
        double It = 0.0;

        if (Iwc > 0.0 || It > 0.0) {
            double Ie = _getEnergyPotential(velocity);
            if (Ie > 0.0) {
                emitters.push_back(DiffuseParticleEmitter(p, velocity, Ie, Iwc, It));
            }
        }
    }
}

void FluidSimulation::_getInsideDiffuseParticleEmitters(
                            std::vector<glm::vec3> &inside, 
                            std::vector<DiffuseParticleEmitter> &emitters) {
    glm::vec3 p;
    for (unsigned int i = 0; i < inside.size(); i++) {
        p = inside[i];
        double It = _getTurbulencePotential(p, _turbulenceField);

        if (It > 0.0) {
            glm::vec3 velocity = _getVelocityAtPosition(p);
            double Ie = _getEnergyPotential(velocity);
            if (Ie > 0.0) {
                emitters.push_back(DiffuseParticleEmitter(p, velocity, Ie, 0.0, It));
            }
        }
    }
}

void FluidSimulation::_getDiffuseParticleEmitters(std::vector<DiffuseParticleEmitter> &emitters) {

    _levelset.calculateSurfaceCurvature();
    _turbulenceField.calculateTurbulenceField(&_MACVelocity, _fluidCellIndices);

    std::vector<glm::vec3> surfaceParticles;
    std::vector<glm::vec3> insideParticles;
    _sortMarkerParticlePositions(surfaceParticles, insideParticles);
    _getSurfaceDiffuseParticleEmitters(surfaceParticles, emitters);
    _getInsideDiffuseParticleEmitters(insideParticles, emitters);

    int wcCount = 0;
    int tCount = 0;

    DiffuseParticleEmitter e;
    for (unsigned int i = 0; i < emitters.size(); i++) {
        e = emitters[i];
        //std::cout << glm::length(e.velocity) << " " << e.energyPotential <<
        //                                        " " << e.wavecrestPotential <<
        //                                        " " << e.turbulencePotential << std::endl;

        if (e.wavecrestPotential > 0) {
            wcCount++;
        }
        if (e.turbulencePotential > 0) {
            tCount++;
        }
    }
    std::cout << "NUM EMITTERS: " << emitters.size() << " " << wcCount << " " << tCount << std::endl;
    std::cout << "NUM DIFFUSE PARTICLE: " << _diffuseParticles.size() << std::endl;
}

int FluidSimulation::_getNumberOfEmissionParticles(DiffuseParticleEmitter &emitter,
                                                   double dt) {
    double wc = _wavecrestEmissionRate*emitter.wavecrestPotential;
    double t = _turbulenceEmissionRate*emitter.turbulencePotential;
    double n = emitter.energyPotential*(wc + t)*dt;

    if (n < 0.0) {
        return 0;
    }

    return (int)(n + 0.5);
}

void FluidSimulation::_emitDiffuseParticles(DiffuseParticleEmitter &emitter, double dt) {
    int n = _getNumberOfEmissionParticles(emitter, dt);

    if (n == 0) {
        return;
    }

    float particleRadius = 4.0f*(float)_markerParticleRadius;
    glm::vec3 axis = glm::normalize(emitter.velocity);

    float eps = 10e-6f;
    glm::vec3 e1;
    if (fabs(axis.x) - 1.0 < eps && fabs(axis.y) < eps && fabs(axis.z) < eps) {
        e1 = glm::normalize(glm::cross(axis, glm::vec3(1.0, 0.0, 0.0)));
    } else {
        e1 = glm::normalize(glm::cross(axis, glm::vec3(0.0, 1.0, 0.0)));
    }

    e1 = e1*(float)particleRadius;
    glm::vec3 e2 = glm::normalize(glm::cross(axis, e1)) * (float)particleRadius;

    float Xr, Xt, Xh, r, theta, h, sinval, cosval, lifetime;
    glm::vec3 p, v;
    GridIndex g;
    for (int i = 0; i < n; i++) {
        Xr = (float)(rand()) / (float)RAND_MAX;
        Xt = (float)(rand()) / (float)RAND_MAX;
        Xh = (float)(rand()) / (float)RAND_MAX;

        r = particleRadius*sqrt(Xr);
        theta = Xt*2.0f*3.141592653f;
        h = Xh*glm::length((float)dt*emitter.velocity);
        sinval = sin(theta);
        cosval = cos(theta);

        p = emitter.position + r*cosval*e1 + r*sinval*e2 + h*axis;
        g = Grid3d::positionToGridIndex(p, _dx);
        if (!Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize) || 
                _isCellSolid(g)) {
            continue;
        }

        v = _getVelocityAtPosition(p);
        lifetime = (float)(emitter.energyPotential*_maxDiffuseParticleLifetime);
        _diffuseParticles.push_back(DiffuseParticle(p, v, lifetime));
    }
}

void FluidSimulation::_emitDiffuseParticles(std::vector<DiffuseParticleEmitter> &emitters,
                                            double dt) {
    for (unsigned int i = 0; i < emitters.size(); i++) {
        _emitDiffuseParticles(emitters[i], dt);
    }
}

int FluidSimulation::_getDiffuseParticleType(DiffuseParticle &dp) {
    double bubbleDist = _minBubbleToSurfaceDistance*_dx;
    double foamDist = _maxFoamToSurfaceDistance*_dx;
    double dist = _levelset.getSignedDistance(dp.position);

    int type;
    if (dist > 0.0) {       // inside surface
        if (dist < bubbleDist) {
            type = DP_FOAM;
        } else {
            type = DP_BUBBLE;
        }
    } else {                // outside surface
        if (fabs(dist) < foamDist) {
            type = DP_FOAM;
        } else {
            type = DP_SPRAY;
        }
    }

    if (type == DP_FOAM) {
        double k = _levelset.getSurfaceCurvature(dp.position);
        if (k < _maxFlatCurvature) {
            type = DP_BUBBLE;
        }
    }

    return type;
}

void FluidSimulation::_updateDiffuseParticleTypesAndVelocities() {
    DiffuseParticle dp;
    int type;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        type = _getDiffuseParticleType(dp);
        _diffuseParticles[i].type = type;

        // Foam particles don't need to calculate their own velocity,
        // but spray and bubble particles do, so reinitialize velocity
        // when transitioning from foam type.
        if (type != DP_FOAM && dp.type == DP_FOAM) {
            glm::vec3 v = _getVelocityAtPosition(dp.position);
            _diffuseParticles[i].velocity = v;
        }
    }
}

void FluidSimulation::_updateDiffuseParticleLifetimes(double dt) {
    std::vector<DiffuseParticle> livingParticles;
    livingParticles.reserve(_diffuseParticles.size());

    DiffuseParticle dp;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        if (dp.type == DP_FOAM) {
            _diffuseParticles[i].lifetime = dp.lifetime - (float)dt;
        }

        if (_diffuseParticles[i].lifetime > 0.0f) {
            livingParticles.push_back(_diffuseParticles[i]);
        }
    }

    int deaths = _diffuseParticles.size() - livingParticles.size();
    std::cout << "PARTICLE DEATHS: " << deaths << std::endl;

    _diffuseParticles = livingParticles;
}

void FluidSimulation::_getNextBubbleDiffuseParticle(DiffuseParticle &dp,
                                                    DiffuseParticle &nextdp,
                                                    double dt) {
    glm::vec3 vmac = _getVelocityAtPosition(dp.position);
    glm::vec3 vbub = dp.velocity;
    glm::vec3 bouyancyVelocity = (float)-_bubbleBouyancyCoefficient*_bodyForce;
    glm::vec3 dragVelocity = (float)_bubbleDragCoefficient*(vmac - vbub) / (float)dt;

    nextdp.velocity = dp.velocity + (float)dt*(bouyancyVelocity + dragVelocity);
    nextdp.position = dp.position + nextdp.velocity*(float)dt;
}

void FluidSimulation::_getNextSprayDiffuseParticle(DiffuseParticle &dp,
                                                   DiffuseParticle &nextdp,
                                                   double dt) {
    nextdp.velocity = dp.velocity +  _bodyForce*(float)dt;
    nextdp.position = dp.position + nextdp.velocity*(float)dt;
}

void FluidSimulation::_getNextFoamDiffuseParticle(DiffuseParticle &dp,
                                                  DiffuseParticle &nextdp,
                                                  double dt) {
    glm::vec3 v0 = _getVelocityAtPosition(dp.position);
    nextdp.velocity = dp.velocity;
    nextdp.position = _RK2(dp.position, v0, dt);
}

void FluidSimulation::_advanceDiffuseParticles(double dt) {
    DiffuseParticle dp, nextdp;
    GridIndex g;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];

        if (dp.type == DP_SPRAY) {
            _getNextSprayDiffuseParticle(dp, nextdp, dt);
        } else if (dp.type == DP_BUBBLE) {
            _getNextBubbleDiffuseParticle(dp, nextdp, dt);
        } else {
            _getNextFoamDiffuseParticle(dp, nextdp, dt);
        }

        g = Grid3d::positionToGridIndex(nextdp.position, _dx);

        if (_isCellSolid(g)) {
            glm::vec3 norm;
            glm::vec3 coll = _calculateSolidCellCollision(dp.position, 
                                                          nextdp.position, &norm);

            // jog p back a bit from cell face
            nextdp.position = coll + (float)(0.01*_dx)*norm;

            glm::vec3 v = nextdp.velocity;
            if (fabs(norm.x) == 1.0) {
                nextdp.velocity = glm::vec3(0.0, v.y, v.z);
            } else if (fabs(norm.y) == 1.0) {
                nextdp.velocity = glm::vec3(v.x, 0.0, v.z);
            } else if (fabs(norm.z) == 1.0) {
                nextdp.velocity = glm::vec3(v.x, v.y, 0.0);
            }
        }

        g = Grid3d::positionToGridIndex(nextdp.position, _dx);
        if (!_isCellSolid(g)) {
            _diffuseParticles[i].position = nextdp.position;
            _diffuseParticles[i].velocity = nextdp.velocity;
        } 
    }
}

void FluidSimulation::_updateDiffuseMaterial(double dt) {
    std::vector<DiffuseParticleEmitter> emitters;
    _getDiffuseParticleEmitters(emitters);
    _emitDiffuseParticles(emitters, dt);

    if (_diffuseParticles.size() == 0.0) {
        return;
    }

    _updateDiffuseParticleTypesAndVelocities();
    _updateDiffuseParticleLifetimes(dt);
    _advanceDiffuseParticles(dt);

    int spraycount = 0;
    int bubblecount = 0;
    int foamcount = 0;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        int type = _diffuseParticles[i].type;
        if (type == DP_SPRAY) {
            spraycount++;
        } else if (type == DP_BUBBLE) {
            bubblecount++;
        } else if (type == DP_FOAM) {
            foamcount++;
        }
    }

    std::cout << "PARTICLE TYPES: SPRAY: " << spraycount << 
                 " BUBBLE: " << bubblecount << 
                 " FOAM: " << foamcount << std::endl; 
}

/********************************************************************************
    UPDATE MARKER PARTICLE VELOCITIES
********************************************************************************/

void FluidSimulation::_updateMarkerParticleVelocities(MACVelocityField &savedField) {
    MarkerParticle p;
    glm::vec3 vPIC, vFLIP;
    glm::vec3 dv;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];

        if (_ratioPICFLIP > 0.0) {
            vPIC = _MACVelocity.evaluateVelocityAtPosition(p.position);
        }
        if (_ratioPICFLIP < 1.0) {
            dv = _MACVelocity.evaluateChangeInVelocityAtPosition(p.position, savedField);
            vFLIP = p.velocity + dv;
        }
        
        _markerParticles[i].velocity = (float)_ratioPICFLIP * vPIC + (float)(1 - _ratioPICFLIP) * vFLIP;
    }
}

/********************************************************************************
    ADVANCE MARKER PARTICLES
********************************************************************************/

glm::vec3 FluidSimulation::_RK2(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = _getVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 p1 = p0 + (float)dt*k2;

    return p1;
}

glm::vec3 FluidSimulation::_RK3(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = _getVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 k3 = _getVelocityAtPosition(p0 + (float)(0.75*dt)*k2);
    glm::vec3 p1 = p0 + (float)(dt/9.0f)*(2.0f*k1 + 3.0f*k2 + 4.0f*k3);

    return p1;
}

glm::vec3 FluidSimulation::_RK4(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = _getVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 k3 = _getVelocityAtPosition(p0 + (float)(0.5*dt)*k2);
    glm::vec3 k4 = _getVelocityAtPosition(p0 + (float)dt*k3);
    glm::vec3 p1 = p0 + (float)(dt/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);

    return p1;
}

bool FluidSimulation::_isPointOnCellFace(glm::vec3 p, CellFace f, double eps) {

    if (fabs(fabs(f.normal.x) - 1.0) < eps) {
        return fabs(p.x - f.minx) < eps &&
               p.y >= f.miny && p.y < f.maxy && p.z >= f.minz && p.z < f.maxz;
    }
    else if (fabs(fabs(f.normal.y) - 1.0) < eps) {
        return fabs(p.y - f.miny) < eps &&
               p.x >= f.minx && p.x < f.maxx && p.z >= f.minz && p.z < f.maxz;
    }
    else if (fabs(fabs(f.normal.z) - 1.0) < eps) {
        return fabs(p.z - f.minz) < eps &&
               p.x >= f.minx && p.x < f.maxx && p.y >= f.miny && p.y < f.maxy;
    }

    return false;
}

FluidSimulation::CellFace FluidSimulation::_getCellFace(int i, int j, int k, 
                                                        glm::vec3 normal) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));
    
    double eps = 10e-4;
    glm::vec3 trans;
    if (fabs(fabs(normal.x) - 1.0) < eps) {
        trans = (float)(0.5 * _dx) * glm::vec3(0.0, 1.0, 1.0);
    }
    else if (fabs(fabs(normal.y) - 1.0) < eps) {
        trans = (float)(0.5 * _dx) * glm::vec3(1.0, 0.0, 1.0);
    }
    else if (fabs(fabs(normal.z) - 1.0) < eps) {
        trans = (float)(0.5 * _dx) * glm::vec3(1.0, 1.0, 0.0);
    }

    glm::vec3 c = Grid3d::GridIndexToCellCenter(i, j, k, _dx);
    glm::vec3 minp = c + (float)(0.5*_dx)*normal - trans;
    glm::vec3 maxp = c + (float)(0.5*_dx)*normal + trans;

    return CellFace(normal, minp, maxp);
}

void FluidSimulation::_getCellFaces(int i, int j, int k, CellFace faces[6]) {
    faces[0] = _getCellFace(i, j, k, glm::vec3(-1.0,  0.0,  0.0));
    faces[1] = _getCellFace(i, j, k, glm::vec3( 1.0,  0.0,  0.0));
    faces[2] = _getCellFace(i, j, k, glm::vec3( 0.0, -1.0,  0.0));
    faces[3] = _getCellFace(i, j, k, glm::vec3( 0.0,  1.0,  0.0));
    faces[4] = _getCellFace(i, j, k, glm::vec3( 0.0,  0.0, -1.0));
    faces[5] = _getCellFace(i, j, k, glm::vec3( 0.0,  0.0,  1.0));
}

std::vector<FluidSimulation::CellFace> FluidSimulation::
             _getNeighbourSolidCellFaces(int i, int j, int k) {

    assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    std::vector<CellFace> faces;
    glm::vec3 normals[6] = { glm::vec3(-1.0, 0.0, 0.0), glm::vec3(1.0, 0.0, 0.0),
                             glm::vec3(0.0, -1.0, 0.0), glm::vec3(0.0, 1.0, 0.0), 
                             glm::vec3(0.0, 0.0, -1.0), glm::vec3(0.0, 0.0, 1.0) };
    GridIndex nc[26];
    Grid3d::getNeighbourGridIndices26(i, j, k, nc);
    for (int idx = 0; idx < 26; idx++) {
        GridIndex c = nc[idx];
        if (Grid3d::isGridIndexInRange(c, _isize, _jsize, _ksize) && 
                _isCellSolid(c)) {
            for (int normidx = 0; normidx < 6; normidx++) {
                faces.push_back(_getCellFace(c.i, c.j, c.k, normals[normidx]));
            }
        }
    }

    return faces;
}

bool FluidSimulation::_getVectorFaceIntersection(glm::vec3 p0, glm::vec3 vnorm, CellFace f, 
                                                 glm::vec3 *intersect) {

    glm::vec3 planep = glm::vec3(f.minx, f.miny, f.minz);
    bool isIntersecting = Collision::lineIntersectsPlane(
        p0, vnorm, planep, f.normal, intersect);

    double eps = 10e-9;
    if (_isPointOnCellFace(*intersect, f, eps)) {
        return true;
    }

    return false;
}

// Check if p lies on a cell face which borders a fluid cell and a solid cell. 
// if so, *f will store the face with normal pointing away from solid
bool FluidSimulation::_isPointOnSolidBoundary(glm::vec3 p, CellFace *f, double eps) {
    int i, j, k;
    Grid3d::positionToGridIndex(p, _dx, &i, &j, &k);

    int U = 0; int V = 1; int W = 2;
    int side;
    int dir;
    CellFace faces[6];
    _getCellFaces(i, j, k, faces);
    for (int idx = 0; idx < 6; idx++) {
        if (_isPointOnCellFace(p, faces[idx], eps)) {
            glm::vec3 n = faces[idx].normal;

            int fi, fj, fk;
            if      (n.x == -1.0) { fi = i;     fj = j;     fk = k;     side = U; dir = -1; }
            else if (n.x == 1.0)  { fi = i + 1; fj = j;     fk = k;     side = U; dir =  1; }
            else if (n.y == -1.0) { fi = i;     fj = j;     fk = k;     side = V; dir = -1; }
            else if (n.y == 1.0)  { fi = i;     fj = j + 1; fk = k;     side = V; dir =  1; }
            else if (n.z == -1.0) { fi = i;     fj = j;     fk = k;     side = W; dir = -1; }
            else if (n.z == 1.0)  { fi = i;     fj = j;     fk = k + 1; side = W; dir =  1; }

            bool isCellSolid = _isCellSolid(i, j, k);
            if      (side == U && _isFaceBorderingMaterialU(fi, fj, fk, M_SOLID)) {
                if (dir == -1) {
                    *f = isCellSolid ? _getCellFace(i, j, k, glm::vec3(-1.0, 0.0, 0.0)) :
                                       _getCellFace(i - 1, j, k, glm::vec3(1.0, 0.0, 0.0));
                }
                else {
                    *f = isCellSolid ? _getCellFace(i, j, k, glm::vec3(1.0, 0.0, 0.0)) :
                                       _getCellFace(i + 1, j, k, glm::vec3(-1.0, 0.0, 0.0));
                }
                return true;
            }
            else if (side == V && _isFaceBorderingMaterialV(fi, fj, fk, M_SOLID)) {
                if (dir == -1) {
                    *f = isCellSolid ? _getCellFace(i, j, k, glm::vec3(0.0, -1.0, 0.0)) :
                                       _getCellFace(i, j - 1, k, glm::vec3(0.0, 1.0, 0.0));
                }
                else {
                    *f = isCellSolid ? _getCellFace(i, j, k, glm::vec3(0.0, 1.0, 0.0)) :
                                       _getCellFace(i, j + 1, k, glm::vec3(0.0, -1.0, 0.0));
                }
                return true;
            }
            else if (side == W && _isFaceBorderingMaterialW(fi, fj, fk, M_SOLID)) {
                if (dir == -1) {
                    *f = isCellSolid ? _getCellFace(i, j, k, glm::vec3(0.0, 0.0, -1.0)) :
                                       _getCellFace(i, j, k - 1, glm::vec3(0.0, 0.0, 1.0));
                }
                else {
                    *f = isCellSolid ? _getCellFace(i, j, k, glm::vec3(0.0, 0.0, 1.0)) :
                                       _getCellFace(i, j, k + 1, glm::vec3(0.0, 0.0, -1.0));
                }
                return true;
            }
        }
    }

    return false;
}

std::vector<FluidSimulation::CellFace> FluidSimulation::
        _getSolidCellFaceCollisionCandidates(int i, int j, int k, glm::vec3 dir) {
    std::vector<CellFace> faces;

    std::vector<CellFace> allfaces = _getNeighbourSolidCellFaces(i, j, k);
    for (unsigned int idx = 0; idx < allfaces.size(); idx++) {
        // must be obtuse angle for a collision
        if (glm::dot(allfaces[idx].normal, dir) < 0) {
            faces.push_back(allfaces[idx]);
        }
    }

    return faces;
}

bool FluidSimulation::_findFaceCollision(glm::vec3 p0, glm::vec3 p1, CellFace *face, glm::vec3 *intersection) {
    int i, j, k;
    Grid3d::positionToGridIndex(p0, _dx, &i, &j, &k);
    glm::vec3 vnorm = glm::normalize(p1 - p0);
    std::vector<CellFace> faces = _getSolidCellFaceCollisionCandidates(i, j, k, vnorm);

    glm::vec3 closestIntersection;
    CellFace closestFace;
    double mindistsq = std::numeric_limits<double>::infinity();
    bool isCollisionFound = false;
    for (unsigned int idx = 0; idx < faces.size(); idx++) {
        CellFace f = faces[idx];

        glm::vec3 intersect;
        bool isIntersecting = _getVectorFaceIntersection(p0, vnorm, f, &intersect);

        if (!isIntersecting) {
            continue;
        }

        glm::vec3 trans = intersect - p0;
        double distsq = glm::dot(trans, trans);
        if (distsq < mindistsq) {
            mindistsq = distsq;
            closestIntersection = intersect;
            closestFace = f;
            isCollisionFound = true;
        }
    }

    if (isCollisionFound) {
        *face = closestFace;
        *intersection = closestIntersection;
    }

    return isCollisionFound;
}

glm::vec3 FluidSimulation::_calculateSolidCellCollision(glm::vec3 p0, 
                                                        glm::vec3 p1, 
                                                        glm::vec3 *normal) {
    int fi, fj, fk, si, sj, sk;
    Grid3d::positionToGridIndex(p0, _dx, &fi, &fj, &fk);
    Grid3d::positionToGridIndex(p1, _dx, &si, &sj, &sk);
    assert(!_isCellSolid(fi, fj, fk));
    assert(_isCellSolid(si, sj, sk));

    // p0 and p1 may not be located in neighbouring cells. Find
    // the neighbouring cell and a point in the cell just before collision 
    // with solid wall. Keep stepping back from p1 until solid collision neighbours
    // are found.
    glm::vec3 vnorm = glm::normalize(p1 - p0);
    int numSteps = 1;
    while (!Grid3d::isGridIndicesNeighbours(fi, fj, fk, si, sj, sk)) {
        glm::vec3 newp = p1 - (float)(_dx - 10e-6)*vnorm;
        int newi, newj, newk;
        Grid3d::positionToGridIndex(newp, _dx, &newi, &newj, &newk);

        if (_isCellSolid(newi, newj, newk)) {
            p1 = newp;
            si = newi; sj = newj; sk = newk;
        }
        else {
            p0 = newp;
            fi = newi, fj = newj, fk = newk;
        }

        numSteps++;
        assert(numSteps < 100);
        assert(!(fi == si && fj == sj && fk == sk));
    }

    CellFace collisionFace;
    glm::vec3 collisionPoint;
    bool isCollisionFound = _findFaceCollision(p0, p1, &collisionFace, &collisionPoint);

    if (isCollisionFound) {
        *normal = collisionFace.normal;
        return collisionPoint;
    }
    else {
        *normal = glm::vec3(0.0, 0.0, 0.0);
        return p0;
    }
}

void FluidSimulation::_advanceRangeOfMarkerParticles(int startIdx, int endIdx, double dt) {
    assert(startIdx <= endIdx);

    MarkerParticle mp;
    glm::vec3 p;
    glm::vec3 vi;
    for (int idx = startIdx; idx <= endIdx; idx++) {
        mp = _markerParticles[idx];

        vi = mp.velocity;
        p = _RK4(mp.position, vi, dt);

        if (!Grid3d::isPositionInGrid(p.x, p.y, p.z, _dx, _isize, _jsize, _ksize)) {
            continue;
        }

        int i, j, k;
        Grid3d::positionToGridIndex(p, _dx, &i, &j, &k);

        glm::vec3 norm;
        if (_isCellSolid(i, j, k)) {
            glm::vec3 coll = _calculateSolidCellCollision(mp.position, p, &norm);

            // jog p back a bit from cell face
            p = coll + (float)(0.01*_dx)*norm;
        }

        Grid3d::positionToGridIndex(p, _dx, &i, &j, &k);
        if (!_isCellSolid(i, j, k)) {
            _markerParticles[idx].position = p;
        }
    }
}

void FluidSimulation::_shuffleMarkerParticleOrder() {
    MarkerParticle mi;
    for (int i = _markerParticles.size() - 2; i >= 0; i--) {
        int j = (rand() % (int)(i - 0 + 1));
        mi = _markerParticles[i];
        _markerParticles[i] = _markerParticles[j];
        _markerParticles[j] = mi;
    }
}

bool compareByMarkerParticlePosition(const MarkerParticle p1, MarkerParticle p2) {
    if (p1.position.x != p2.position.x) { return p1.position.x < p2.position.x; }
    if (p1.position.y != p2.position.y) { return p1.position.y < p2.position.y; }
    if (p1.position.z != p2.position.z) { return p1.position.z < p2.position.z; }
    return false;
}

void FluidSimulation::_sortMarkerParticlesByGridIndex() {
    std::sort(_markerParticles.begin(), _markerParticles.end(), compareByMarkerParticlePosition);
}

void FluidSimulation::_removeMarkerParticles() {
    double maxspeed = (_CFLConditionNumber*_dx) / _minTimeStep;
    double maxspeedsq = maxspeed*maxspeed;

    Array3d<int> countGrid = Array3d<int>(_isize, _jsize, _ksize, 0);
    _shuffleMarkerParticleOrder();

    std::vector<MarkerParticle> aliveParticles;
    aliveParticles.reserve(_markerParticles.size());

    MarkerParticle mp;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        mp = _markerParticles[i];

        double speedsq = glm::dot(mp.velocity, mp.velocity);
        if (speedsq > maxspeedsq) {
            continue;
        }

        g = Grid3d::positionToGridIndex(mp.position, _dx);
        if (countGrid(g) >= _maxMarkerParticlesPerCell) {
            continue;
        }
        countGrid.add(g, 1);

        aliveParticles.push_back(mp);
    }

    int dead = _markerParticles.size() - aliveParticles.size();
    std::cout << "\t\tDEAD: " << dead << std::endl;

    _markerParticles = aliveParticles;
    _sortMarkerParticlesByGridIndex();
}

void FluidSimulation::_advanceMarkerParticles(double dt) {
    int size = (int)_markerParticles.size();

    std::vector<int> startIndices;
    std::vector<int> endIndices;

    int numThreads = _numAdvanceMarkerParticleThreads;
    int chunksize = (int)floor(size / numThreads);
    for (int i = 0; i < numThreads; i++) {
        int startIdx = (i == 0) ? 0 : endIndices[i - 1] + 1;
        int endIdx = (i == numThreads - 1) ? size - 1 : startIdx + chunksize - 1;

        startIndices.push_back(startIdx);
        endIndices.push_back(endIdx);
    }

    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; i++) {
        threads.push_back(std::thread(&FluidSimulation::_advanceRangeOfMarkerParticles,
                                      this,
                                      startIndices[i],
                                      endIndices[i],
                                      dt));
    }

    for (int i = 0; i < numThreads; i++) {
        threads[i].join();
    }

    _removeMarkerParticles();
}

/********************************************************************************
    TIME STEP
********************************************************************************/

void FluidSimulation::_stepFluid(double dt) {
    _simulationTime += dt;

    StopWatch timer1 = StopWatch();
    StopWatch timer2 = StopWatch();
    StopWatch timer3 = StopWatch();
    StopWatch timer4 = StopWatch();
    StopWatch timer5 = StopWatch();
    StopWatch timer6 = StopWatch();
    StopWatch timer7 = StopWatch();
    StopWatch timer8 = StopWatch();
    StopWatch timer9 = StopWatch();
    StopWatch timer10 = StopWatch();
    StopWatch timer11 = StopWatch();
    StopWatch timer12 = StopWatch();
    StopWatch timer13 = StopWatch();

    _logfile.separator();
    _logfile.timestamp();
    _logfile.newline();
    _logfile.log("Frame: ", _currentFrame, 0);
    _logfile.log("StepTime: ", dt, 4);
    _logfile.newline();

    timer1.start();

    timer2.start();
    _updateFluidCells();
    timer2.stop();

    _logfile.log("Update Fluid Cells:          \t", timer4.getTime(), 4);
    _logfile.log("Num Fluid Cells: \t", (int)_fluidCellIndices.size(), 4, 1);

    timer3.start();
    _reconstructFluidSurface();
    timer3.stop();

    _logfile.log("Reconstruct Fluid Surface:  \t", timer3.getTime(), 4);

    timer4.start();
    _updateLevelSetSignedDistance();
    timer4.stop();

    _logfile.log("Update Level set:           \t", timer4.getTime(), 4);

    timer5.start();
    if (_isLastTimeStepForFrame) {
        _reconstructOutputFluidSurface(_frameTimeStep);
    }
    timer5.stop();

    _logfile.log("Reconstruct Output Surface: \t", timer5.getTime(), 4);

    timer6.start();
    _advectVelocityField();
    timer6.stop();

    _logfile.log("Advect Velocity Field:       \t", timer6.getTime(), 4);

    timer7.start();
    _applyBodyForcesToVelocityField(dt);
    {
        MACVelocityField savedMACVelocity = _MACVelocity;
        timer7.stop();

        _logfile.log("Apply Body Forces:           \t", timer7.getTime(), 4);

        timer8.start();
        {
            Array3d<float> pressureGrid = Array3d<float>(_isize, _jsize, _ksize, 0.0f);
            _updatePressureGrid(pressureGrid, dt);
            timer8.stop();

            _logfile.log("Update Pressure Grid:        \t", timer8.getTime(), 4);

            timer9.start();
            _applyPressureToVelocityField(pressureGrid, dt);
        }
        timer9.stop();

        _logfile.log("Apply Pressure:              \t", timer9.getTime(), 4);

        timer10.start();
        _extrapolateFluidVelocities();
        timer10.stop();

        _logfile.log("Extrapolate Fluid Velocities:\t", timer10.getTime(), 4);

        timer11.start();
        if (_isDiffuseMaterialOutputEnabled) {
            _updateDiffuseMaterial(dt);
        }
        timer11.stop();

        _logfile.log("Update Diffuse Material:     \t", timer11.getTime(), 4);

        timer12.start();
        _updateMarkerParticleVelocities(savedMACVelocity);
    }
    timer12.stop();

    _logfile.log("Update PIC/FLIP velocities   \t", timer12.getTime(), 4);

    timer13.start();
    _advanceMarkerParticles(dt);
    timer13.stop();

    _logfile.log("Advance Marker Particles:    \t", timer13.getTime(), 4);

    timer1.stop();

    double totalTime = floor(timer1.getTime()*1000.0) / 1000.0;
    _realTime += totalTime;
    _logfile.newline();
    _logfile.log("Update Time:           \t", totalTime, 3, 1);
    _logfile.newline();

    double p2 = floor(1000 * timer2.getTime() / totalTime) / 10.0;
    double p3 = floor(1000 * timer3.getTime() / totalTime) / 10.0;
    double p4 = floor(1000 * timer4.getTime() / totalTime) / 10.0;
    double p5 = floor(1000 * timer5.getTime() / totalTime) / 10.0;
    double p6 = floor(1000 * timer6.getTime() / totalTime) / 10.0;
    double p7 = floor(1000 * timer7.getTime() / totalTime) / 10.0;
    double p8 = floor(1000 * timer8.getTime() / totalTime) / 10.0;
    double p9 = floor(1000 * timer9.getTime() / totalTime) / 10.0;
    double p10 = floor(1000 * timer10.getTime() / totalTime) / 10.0;
    double p11 = floor(1000 * timer11.getTime() / totalTime) / 10.0;
    double p12 = floor(1000 * timer12.getTime() / totalTime) / 10.0;
    double p13 = floor(1000 * timer13.getTime() / totalTime) / 10.0;

    _logfile.log("---Percentage Breakdown---", "");
    _logfile.log("Update Fluid Cells:          \t", p2, 3);
    _logfile.log("Reconstruct Fluid Surface    \t", p3, 3);
    _logfile.log("Update Level Set:            \t", p4, 3);
    _logfile.log("Reconstruct Output Surface   \t", p5, 3);
    _logfile.log("Advect Velocity Field:       \t", p6, 3);
    _logfile.log("Apply Body Forces:           \t", p7, 3);
    _logfile.log("Update Pressure Grid:        \t", p8, 3);
    _logfile.log("Apply Pressure:              \t", p9, 3);
    _logfile.log("Extrapolate Fluid Velocities:\t", p10, 3);
    _logfile.log("Update Diffuse Material:     \t", p11, 3);
    _logfile.log("Update PIC/FLIP Velocity:    \t", p12, 3);
    _logfile.log("Advance Marker Particles:    \t", p13, 3);
    _logfile.newline();

    _logfile.log("Simulation time: ", _simulationTime, 3);
    _logfile.log("Real time: ", _realTime, 2);
    _logfile.newline();
    _logfile.write();
}

double FluidSimulation::_getMaximumMarkerParticleSpeed() {
    double maxsq = 0.0;
    MarkerParticle mp;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        double distsq = glm::dot(mp.velocity, mp.velocity);
        if (distsq > maxsq) {
            maxsq = distsq;
        }
    }

    return sqrt(maxsq);
}

double FluidSimulation::_calculateNextTimeStep() {
    double maxu = _getMaximumMarkerParticleSpeed();
    double timeStep = _CFLConditionNumber*_dx / maxu;

    timeStep = (double)fmax((float)_minTimeStep, (float)timeStep);
    timeStep = (double)fmin((float)_maxTimeStep, (float)timeStep);

    return timeStep;
}

void FluidSimulation::update(double dt) {
    if (!_isSimulationRunning || !_isSimulationInitialized) {
        return;
    }
    _isCurrentFrameFinished = false;

    _frameTimeStep = dt;

    saveState();

    _currentTimeStep = 0;
    double timeleft = dt;
    while (timeleft > 0.0) {
        double timestep = _calculateNextTimeStep();
        if (timeleft - timestep < 0.0) {
            timestep = timeleft;
        }
        timeleft -= timestep;

        double eps = 10e-9;
        _isLastTimeStepForFrame = fabs(timeleft) < eps;

        _stepFluid(timestep);

        _currentTimeStep++;
    }
    _currentFrame++;

    _isCurrentFrameFinished = true;
}