#include "fluidsimulation.h"


FluidSimulation::FluidSimulation() :
                                _bodyForce(glm::vec3(0.0, 0.0, 0.0)),
                                _MACVelocity(_i_voxels, _j_voxels, _k_voxels, _dx),
                                _materialGrid(Array3d<int>(_i_voxels, _j_voxels, _k_voxels, M_AIR)),
                                _pressureGrid(Array3d<double>(_i_voxels, _j_voxels, _k_voxels, 0.0)),
                                _layerGrid(Array3d<int>(_i_voxels, _j_voxels, _k_voxels, -1)),
                                _matrixA(_i_voxels, _j_voxels, _k_voxels),
                                _preconditioner(_i_voxels, _j_voxels, _k_voxels),
                                _implicitFluidScalarField(_i_voxels, _j_voxels, _k_voxels, _dx),
                                _levelsetField(_i_voxels, _j_voxels, _k_voxels, _dx),
                                _levelset(_i_voxels, _j_voxels, _k_voxels, _dx),
                                _markerParticleRadius(pow(3*(_dx*_dx*_dx / 8.0) / (4*3.141592653), 1.0/3.0))
{
}

FluidSimulation::FluidSimulation(int x_voxels, int y_voxels, int z_voxels, double cell_size) :
                                _i_voxels(x_voxels), _j_voxels(y_voxels), _k_voxels(z_voxels), _dx(cell_size),
                                _bodyForce(glm::vec3(0.0, 0.0, 0.0)),
                                _MACVelocity(x_voxels, y_voxels, z_voxels, cell_size),
                                _materialGrid(Array3d<int>(x_voxels, y_voxels, z_voxels, M_AIR)),
                                _pressureGrid(Array3d<double>(x_voxels, y_voxels, z_voxels, 0.0)),
                                _layerGrid(Array3d<int>(x_voxels, y_voxels, z_voxels, -1)),
                                _matrixA(x_voxels, y_voxels, z_voxels),
                                _preconditioner(x_voxels, y_voxels, z_voxels),
                                _GridIndexToEigenVectorXdIndex(x_voxels, y_voxels, z_voxels, -1),
                                _implicitFluidScalarField(x_voxels + 1, y_voxels + 1, z_voxels + 1, cell_size),
                                _levelsetField(x_voxels, y_voxels, z_voxels, cell_size),
                                _levelset(x_voxels, y_voxels, z_voxels, cell_size),
                                _markerParticleRadius(pow(3*(_dx*_dx*_dx / 8.0) / (4*3.141592653), 1.0/3.0))
{
    _materialGrid.setOutOfRangeValue(M_SOLID);
    _implicitFluidScalarField.enableCellCenterValues();
    _logfile = LogFile();
}

FluidSimulation::FluidSimulation(FluidSimulationSaveState &state) {
    assert(state.isLoadStateInitialized());
    _initializeSimulationFromSaveState(state);

    _materialGrid.setOutOfRangeValue(M_SOLID);
    _logfile = LogFile();
}

FluidSimulation::~FluidSimulation() {
    for (int i = 0; i < (int)_fluidSources.size(); i++) {
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

void FluidSimulation::addBodyForce(glm::vec3 f) {
    _bodyForce += f;
}

void FluidSimulation::setBodyForce(glm::vec3 f) {
    _bodyForce = f;
}

void FluidSimulation::addImplicitFluidPoint(glm::vec3 p, double r) {
    if (_fluidInitializationType == MESH) {
        return;
    }

    _implicitFluidScalarField.addPoint(p, r);
    _fluidInitializationType = IMPLICIT;
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

    _implicitFluidScalarField.addCuboid(p, w, h, d);
    _fluidInitializationType = IMPLICIT;
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
    _fluidSources.push_back(source);
    _sphericalFluidSources.push_back(source);
    return source;
}

SphericalFluidSource* FluidSimulation::addSphericalFluidSource(glm::vec3 pos, double r, 
                                             glm::vec3 velocity) {
    SphericalFluidSource *source = new SphericalFluidSource(pos, r, velocity);
    _fluidSources.push_back(source);
    _sphericalFluidSources.push_back(source);
    return source;
}

CuboidFluidSource* FluidSimulation::addCuboidFluidSource(AABB bbox) {
    CuboidFluidSource *source = new CuboidFluidSource(bbox);
    _fluidSources.push_back(source);
    _cuboidFluidSources.push_back(source);
    return source;
}

CuboidFluidSource* FluidSimulation::addCuboidFluidSource(AABB bbox, glm::vec3 velocity) {
    CuboidFluidSource *source = new CuboidFluidSource(bbox, velocity);
    _fluidSources.push_back(source);
    _cuboidFluidSources.push_back(source);
    return source;
}

void FluidSimulation::addSolidCell(int i, int j, int k) {
    if (_isSimulationInitialized) { return; }
    assert(Grid3d::isGridIndexInRange(i, j, k, _i_voxels, _j_voxels, _k_voxels));
    _materialGrid.set(i, j, k, M_SOLID);
}

void FluidSimulation::addSolidCells(std::vector<glm::vec3> indices) {
    for (int i = 0; i < (int)indices.size(); i++) {
        addSolidCell((int)indices[i].x, (int)indices[i].y, (int)indices[i].z);
    }
}

void FluidSimulation::removeSolidCell(int i, int j, int k) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _i_voxels, _j_voxels, _k_voxels));

    // Cannot remove border cells
    if (Grid3d::isGridIndexOnBorder(i, j, k, _i_voxels, _j_voxels, _k_voxels)) { 
        return; 
    }

    if (_isCellSolid(i, j, k)) {
        _materialGrid.set(i, j, k, M_AIR);
    }
}

void FluidSimulation::removeSolidCells(std::vector<glm::vec3> indices) {
    for (int i = 0; i < (int)indices.size(); i++) {
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

std::vector<glm::vec3> FluidSimulation::getMarkerParticles(int skip) {
    std::vector<glm::vec3> particles;
    particles.reserve(_markerParticles.size());

    for (int i = 0; i < (int)_markerParticles.size(); i += skip) {
        particles.push_back(_markerParticles[i].position);
    }

    return particles;
}

std::vector<glm::vec3> FluidSimulation::getMarkerParticles() {
    return getMarkerParticles(1);
}

std::vector<DiffuseParticle> FluidSimulation::getDiffuseParticles() {
    return _diffuseParticles;
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

/********************************************************************************
    INITIALIZATION
********************************************************************************/
void FluidSimulation::_initializeSolidCells() {
    // fill borders with solid cells
    for (int j = 0; j < _j_voxels; j++) {
        for (int i = 0; i < _i_voxels; i++) {
            _materialGrid.set(i, j, 0, M_SOLID);
            _materialGrid.set(i, j, _k_voxels-1, M_SOLID);
        }
    }

    for (int k = 0; k < _k_voxels; k++) {
        for (int i = 0; i < _i_voxels; i++) {
            _materialGrid.set(i, 0, k, M_SOLID);
            _materialGrid.set(i, _j_voxels-1, k, M_SOLID);
        }
    }

    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            _materialGrid.set(0, j, k, M_SOLID);
            _materialGrid.set(_i_voxels-1, j, k, M_SOLID);
        }
    }
}

void FluidSimulation::_addMarkerParticlesToCell(GridIndex g) {
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
        _markerParticles.push_back(MarkerParticle(p, g.i, g.j, g.k));
    }
}

void FluidSimulation::_initializeFluidMaterial() {
    _isFluidInSimulation = _fluidInitializationType == MESH ||
                           _fluidInitializationType == IMPLICIT;

    if (!_isFluidInSimulation) {
        return;
    }

    std::vector<GridIndex> fluidCells;
    if (_fluidInitializationType == IMPLICIT) {
        // Polygonizer needs an estimate of cells that are inside the surface
        for (int k = 0; k < _materialGrid.depth; k++) {
            for (int j = 0; j < _materialGrid.height; j++) {
                for (int i = 0; i < _materialGrid.width; i++) {
                    GridIndex g(i, j, k);

                    if (_implicitFluidScalarField.isCellInsideSurface(i, j, k) && _isCellAir(g)) {
                        fluidCells.push_back(g);
                    }
                }
            }
        }

        assert(fluidCells.size() > 0);

        Polygonizer3d polygonizer(_implicitFluidScalarField);

        _implicitFluidScalarField.setMaterialGrid(_materialGrid);
        polygonizer.setInsideCellIndices(fluidCells);
        fluidCells.clear();

        polygonizer.polygonizeSurface();
        _surfaceMesh = polygonizer.getTriangleMesh();
        _surfaceMesh.setGridDimensions(_i_voxels, _j_voxels, _k_voxels, _dx);
        _surfaceMesh.getCellsInsideMesh(fluidCells);

    } else if (_fluidInitializationType == MESH) {
        bool success = _surfaceMesh.loadOBJ(_fluidMeshFilename, _fluidMeshOffset, _fluidMeshScale);
        assert(success);
        _surfaceMesh.setGridDimensions(_i_voxels, _j_voxels, _k_voxels, _dx);
        _surfaceMesh.getCellsInsideMesh(fluidCells);

        Polygonizer3d levelsetPolygonizer = Polygonizer3d(&_levelsetField);
        _levelset.setSurfaceMesh(_surfaceMesh);
        _levelset.calculateSignedDistanceField();
        _levelsetField.setMaterialGrid(_materialGrid);
        _levelsetField.setSignedDistanceField(_levelset.getSignedDistanceField());
        levelsetPolygonizer.setInsideCellIndices(fluidCells);
        levelsetPolygonizer.polygonizeSurface();

        fluidCells.clear();
        _surfaceMesh = levelsetPolygonizer.getTriangleMesh();
        _surfaceMesh.setGridDimensions(_i_voxels, _j_voxels, _k_voxels, _dx);
        _surfaceMesh.getCellsInsideMesh(fluidCells);
    }


    GridIndex g;
    std::vector<GridIndex> indices;
    for (int i = 0; i < (int)fluidCells.size(); i++) {
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

    // objects are no longer needed after simulation is initialized
    _implicitFluidScalarField = ImplicitSurfaceScalarField();
    _levelsetField = LevelSetField();

    _isSimulationInitialized = true;
}

void FluidSimulation::_initializeFluidMaterialParticlesFromSaveState() {
    MarkerParticle p;
    for (int i = 0; i < (int)_markerParticles.size(); i++) {
        p = _markerParticles[i];
        assert(!_isCellSolid(p.index));
        _materialGrid.set(p.index, M_FLUID);
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
    std::vector<glm::vec3> positions = state.getMarkerParticles();

    glm::vec3 p;
    GridIndex g;
    for (int i = 0; i < (int)positions.size(); i++) {
        p = positions[i];
        g = Grid3d::positionToGridIndex(p, _dx);
        _markerParticles.push_back(MarkerParticle(p, g.i, g.j, g.k));
    }
}

void FluidSimulation::_initializeSolidCellsFromSaveState(FluidSimulationSaveState &state) {
    std::vector<GridIndex> indices = state.getSolidCellIndices();
    GridIndex g;
    for (int i = 0; i < (int)indices.size(); i++) {
        g = indices[i];
        addSolidCell(g.i, g.j, g.k);
    }
}

void FluidSimulation::_initializeMACGridFromSaveState(FluidSimulationSaveState &state) {
    Array3d<double> U, V, W;
    state.getVelocityField(U, V, W);

    assert(U.width == _i_voxels + 1 && U.height == _j_voxels && U.depth == _k_voxels);
    assert(V.width == _i_voxels && V.height == _j_voxels + 1 && V.depth == _k_voxels);
    assert(W.width == _i_voxels && W.height == _j_voxels && W.depth == _k_voxels + 1);

    _MACVelocity = MACVelocityField(_i_voxels, _j_voxels, _k_voxels, _dx);
    _MACVelocity.setU(U);
    _MACVelocity.setV(V);
    _MACVelocity.setW(W);
}

void FluidSimulation::_initializeSimulationFromSaveState(FluidSimulationSaveState &state) {
    state.getGridDimensions(&_i_voxels, &_j_voxels, &_k_voxels);
    _dx = state.getCellSize();
    _currentFrame = state.getCurrentFrame();

    _bodyForce = glm::vec3(0.0, 0.0, 0.0);
    _materialGrid = Array3d<int>(_i_voxels, _j_voxels, _k_voxels, M_AIR);
    _pressureGrid = Array3d<double>(_i_voxels, _j_voxels, _k_voxels, 0.0);
    _layerGrid = Array3d<int>(_i_voxels, _j_voxels, _k_voxels, -1);
    _matrixA = MatrixCoefficients(_i_voxels, _j_voxels, _k_voxels);
    _preconditioner = VectorCoefficients(_i_voxels, _j_voxels, _k_voxels);
    _GridIndexToEigenVectorXdIndex = Array3d<int>(_i_voxels, _j_voxels, _k_voxels, -1);
    _levelset = LevelSet(_i_voxels, _j_voxels, _k_voxels, _dx);
    _markerParticleRadius = pow(3*(_dx*_dx*_dx / 8.0) / (4*3.141592653), 1.0/3.0);

    _initializeSolidCellsFromSaveState(state);
    _initializeMarkerParticlesFromSaveState(state);
    _initializeFluidMaterialParticlesFromSaveState();
    _initializeMACGridFromSaveState(state);

    _isFluidInSimulation = _fluidCellIndices.size() > 0;
    _isSimulationInitialized = true;
}

/********************************************************************************
    UPDATE FLUID CELLS
********************************************************************************/

void FluidSimulation::_removeMarkerParticlesFromCells(std::vector<GridIndex> &cells) {
    std::vector<MarkerParticle> newv;
    newv.reserve(_markerParticles.size());

    MarkerParticle p;
    for (int i = 0; i < (int)_markerParticles.size(); i++) {
        p = _markerParticles[i];
        if (!_isIndexInList(p.index, cells)) {
            newv.push_back(p);
        }
    }

    _markerParticles = newv;
}

void FluidSimulation::_setVelocitiesForNewFluidCell(GridIndex g, glm::vec3 v) {
    int i = g.i; int j = g.j; int k = g.k;

    if (!_isFaceBorderingMaterialU(i, j, k, M_FLUID) &&
        !_isFaceBorderingMaterialU(i, j, k, M_SOLID)) {
        _MACVelocity.setU(i, j, k, v.x);
    }
    if (!_isFaceBorderingMaterialU(i + 1, j, k, M_FLUID) &&
        !_isFaceBorderingMaterialU(i + 1, j, k, M_SOLID)) {
        _MACVelocity.setU(i + 1, j, k, v.x);
    }

    if (!_isFaceBorderingMaterialV(i, j, k, M_FLUID) &&
        !_isFaceBorderingMaterialV(i, j, k, M_SOLID)) {
        _MACVelocity.setV(i, j, k, v.y);
    }
    if (!_isFaceBorderingMaterialV(i, j + 1, k, M_FLUID) &&
        !_isFaceBorderingMaterialV(i, j + 1, k, M_SOLID)) {
        _MACVelocity.setV(i, j + 1, k, v.y);
    }

    if (!_isFaceBorderingMaterialW(i, j, k, M_FLUID) &&
        !_isFaceBorderingMaterialW(i, j, k, M_SOLID)) {
        _MACVelocity.setW(i, j, k, v.z);
    }
    if (!_isFaceBorderingMaterialW(i, j + 1, k, M_FLUID) &&
        !_isFaceBorderingMaterialW(i, j + 1, k, M_SOLID)) {
        _MACVelocity.setW(i, j + 1, k, v.z);
    }
}

void FluidSimulation::_addNewFluidCells(std::vector<GridIndex> &cells, 
                                        glm::vec3 velocity) {
    GridIndex g;
    for (int i = 0; i < (int)cells.size(); i++) {
        _setVelocitiesForNewFluidCell(cells[i], velocity);
        _addMarkerParticlesToCell(cells[i]);
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
    for (int i = 0; i < (int)_fluidSources.size(); i++) {
        _updateFluidSource(_fluidSources[i]);
    }
}

void FluidSimulation::_updateFluidCells() {
    _updateFluidSources();

    _materialGrid.set(_fluidCellIndices, M_AIR);
    _fluidCellIndices.clear();
    
    MarkerParticle p;
    for (int i = 0; i < (int)_markerParticles.size(); i++) {
        p = _markerParticles[i];
        assert(!_isCellSolid(p.index));
        _materialGrid.set(p.index, M_FLUID);
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

void FluidSimulation::_writeSurfaceMeshToFile() {
    std::string s;
    std::string currentFrame = std::to_string(_currentFrame);
    currentFrame.insert(currentFrame.begin(), 6 - currentFrame.size(), '0');
    s = "bakefiles/" + currentFrame + ".ply";

    _surfaceMesh.writeMeshToPLY(s);

    TriangleMesh bubbleMesh;
    TriangleMesh foamMesh;
    TriangleMesh sprayMesh;
    DiffuseParticle dp;
    for (int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        if (dp.type == DP_BUBBLE) {
            bubbleMesh.vertices.push_back(dp.position);
        } else if (dp.type = DP_FOAM) {
            foamMesh.vertices.push_back(dp.position);
        } else if (dp.type = DP_SPRAY) {
            sprayMesh.vertices.push_back(dp.position);
        }
    }

    bubbleMesh.writeMeshToPLY("bakefiles/bubble" + currentFrame + ".ply");
    foamMesh.writeMeshToPLY("bakefiles/foam" + currentFrame + ".ply");
    sprayMesh.writeMeshToPLY("bakefiles/spray" + currentFrame + ".ply");
}

void FluidSimulation::_reconstructFluidSurface() {
    StopWatch t1 = StopWatch();
    StopWatch t2 = StopWatch();
    StopWatch t3 = StopWatch();
    StopWatch t4 = StopWatch();

    t1.start();
    ImplicitSurfaceScalarField field = ImplicitSurfaceScalarField(_i_voxels + 1, 
                                                                  _j_voxels + 1, 
                                                                  _k_voxels + 1, _dx);
    field.setMaterialGrid(_materialGrid);

    double r = _markerParticleRadius*_markerParticleScale;
    field.setPointRadius(r);

    glm::vec3 p;
    for (int i = 0; i < (int)_markerParticles.size(); i++) {
        p = _markerParticles[i].position;
        field.addPoint(p);
    }

    Polygonizer3d polygonizer = Polygonizer3d(field);
    polygonizer.setInsideCellIndices(_fluidCellIndices);

    polygonizer.polygonizeSurface();
    _surfaceMesh = polygonizer.getTriangleMesh();

    _surfaceMesh.smooth(_surfaceReconstructionSmoothingValue, 
                        _surfaceReconstructionSmoothingIterations);
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
    EXTRAPOLATE FLUID VELOCITIES
********************************************************************************/

void FluidSimulation::_getNeighbourGridIndices6(int i, int j, int k, GridIndex n[6]) {
    n[0] = GridIndex(i-1, j, k);
    n[1] = GridIndex(i+1, j, k);
    n[2] = GridIndex(i, j-1, k);
    n[3] = GridIndex(i, j+1, k);
    n[4] = GridIndex(i, j, k-1);
    n[5] = GridIndex(i, j, k+1);
}

void FluidSimulation::_getNeighbourGridIndices26(int i, int j, int k, GridIndex n[26]) {
    int idx = 0;
    for (int nk = k-1; nk <= k+1; nk++) {
        for (int nj = j-1; nj <= j+1; nj++) {
            for (int ni = i-1; ni <= i+1; ni++) {
                if (!(ni == i && nj == j && nk == k)) {
                    n[idx] = GridIndex(ni, nj, nk);
                    idx++;
                }
            }
        }
    }
}

void FluidSimulation::_updateExtrapolationLayer(int layerIndex) {
    GridIndex neighbours[6];
    GridIndex n;

    for (int k = 0; k < _layerGrid.depth; k++) {
        for (int j = 0; j < _layerGrid.height; j++) {
            for (int i = 0; i < _layerGrid.width; i++) {
                if (_layerGrid(i, j, k) == layerIndex - 1 && !_isCellSolid(i, j, k)) {
                    _getNeighbourGridIndices6(i, j, k, neighbours);
                    for (int idx = 0; idx < 6; idx++) {
                        n = neighbours[idx];

                        if (Grid3d::isGridIndexInRange(n, _i_voxels, _j_voxels, _k_voxels) && 
                                _layerGrid(n) == -1 && !_isCellSolid(n)) {
                            _layerGrid.set(n, layerIndex);
                        }
                    }
                }
            }
        }
    }
}

int FluidSimulation::_updateExtrapolationLayers() {
    _layerGrid.fill(-1);

    GridIndex idx;
    for (int i = 0; i < (int)_fluidCellIndices.size(); i++) {
        idx = _fluidCellIndices[i];
        _layerGrid.set(idx, 0);
    }

    // add 2 extra layers to account for extra values needed during cubic 
    // interpolation calculations
    int numLayers = (int)ceil(_CFLConditionNumber) + 2;
    for (int layer = 1; layer <= numLayers; layer++) {
        _updateExtrapolationLayer(layer);
    }

    return numLayers;
}

double FluidSimulation::_getExtrapolatedVelocityForFaceU(int i, int j, int k, int layerIdx) {

    // First two layers are calculate by averaging neighbours so that values exist for tricubic
    // interpolation at the fluid boundary for layers > 2
    if (layerIdx > 2) {
        glm::vec3 pos = _MACVelocity.velocityIndexToPositionU(i, j, k);
        glm::vec3 v = _getVelocityAtNearestPointOnFluidSurface(pos);
        return v.x;
    }

    GridIndex n[6];
    _getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (_MACVelocity.isIndexInRangeU(c) && _isFaceBorderingLayerIndexU(c, layerIdx - 1)) {
                sum += _MACVelocity.U(c);
                weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

double FluidSimulation::_getExtrapolatedVelocityForFaceV(int i, int j, int k, int layerIdx) {

    if (layerIdx > 2) {
        glm::vec3 pos = _MACVelocity.velocityIndexToPositionV(i, j, k);
        glm::vec3 v = _getVelocityAtNearestPointOnFluidSurface(pos);
        return v.y;
    }

    GridIndex n[6];
    _getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (_MACVelocity.isIndexInRangeV(c) && _isFaceBorderingLayerIndexV(c, layerIdx - 1)) {
            sum += _MACVelocity.V(c);
            weightsum++;
        }
    }

    if (sum == 0.0) {
        return 0.0;
    }

    return sum / weightsum;
}

double FluidSimulation::_getExtrapolatedVelocityForFaceW(int i, int j, int k, int layerIdx) {

    if (layerIdx > 2) {
        glm::vec3 pos = _MACVelocity.velocityIndexToPositionW(i, j, k);
        glm::vec3 v = _getVelocityAtNearestPointOnFluidSurface(pos);
        return v.z;
    }

    GridIndex n[6];
    _getNeighbourGridIndices6(i, j, k, n);

    GridIndex c;
    double sum = 0.0;
    double weightsum = 0.0;

    for (int idx = 0; idx < 6; idx++) {
        c = n[idx];
        if (_MACVelocity.isIndexInRangeW(c) && _isFaceBorderingLayerIndexW(c, layerIdx - 1)) {
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
    return _MACVelocity.evaluateVelocityAtPosition(p);
}

void FluidSimulation::_extrapolateVelocitiesForLayerIndex(int idx) {
    _MACVelocity.resetTemporaryVelocityField();

    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels + 1; i++) {
                if (_isFaceBorderingLayerIndexU(i, j, k, idx) &&
                    !_isFaceBorderingLayerIndexU(i, j, k, idx-1) &&
                    !_isFaceBorderingMaterialU(i, j, k, M_SOLID)) {
                    double v = _getExtrapolatedVelocityForFaceU(i, j, k, idx);
                    _MACVelocity.setTempU(i, j, k, v);
                }
            }
        }
    }

    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels + 1; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (_isFaceBorderingLayerIndexV(i, j, k, idx) &&
                    !_isFaceBorderingLayerIndexV(i, j, k, idx - 1) &&
                    !_isFaceBorderingMaterialV(i, j, k, M_SOLID)) {
                    double v = _getExtrapolatedVelocityForFaceV(i, j, k, idx);
                    _MACVelocity.setTempV(i, j, k, v);
                }
            }
        }
    }

    for (int k = 0; k < _k_voxels + 1; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (_isFaceBorderingLayerIndexW(i, j, k, idx) &&
                    !_isFaceBorderingLayerIndexW(i, j, k, idx - 1) &&
                    !_isFaceBorderingMaterialW(i, j, k, M_SOLID)) {
                    double v = _getExtrapolatedVelocityForFaceW(i, j, k, idx);
                    _MACVelocity.setTempW(i, j, k, v);
                }
            }
        }
    }

    _MACVelocity.commitTemporaryVelocityFieldValues();
}

void FluidSimulation::_resetExtrapolatedFluidVelocities() {
    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels + 1; i++) {
                if (!_isFaceBorderingMaterialU(i, j, k, M_FLUID)) {
                    _MACVelocity.setU(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels + 1; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (!_isFaceBorderingMaterialV(i, j, k, M_FLUID)) {
                    _MACVelocity.setV(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _k_voxels + 1; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (!_isFaceBorderingMaterialW(i, j, k, M_FLUID)) {
                    _MACVelocity.setW(i, j, k, 0.0);
                }
            }
        }
    }
}

void FluidSimulation::_extrapolateFluidVelocities() {
    _resetExtrapolatedFluidVelocities();
    int numLayers = _updateExtrapolationLayers();

    for (int i = 1; i <= numLayers; i++) {
        _extrapolateVelocitiesForLayerIndex(i);
    }
}

/********************************************************************************
    APPLY BODY FORCES
********************************************************************************/

void FluidSimulation::_applyBodyForcesToVelocityField(double dt) {
    if (fabs(_bodyForce.x) > 0.0) {
        for (int k = 0; k < _k_voxels; k++) {
            for (int j = 0; j < _j_voxels; j++) {
                for (int i = 0; i < _i_voxels + 1; i++) {
                    if (_isFaceBorderingMaterialU(i, j, k, M_FLUID) ||
                            _isFaceVelocityExtrapolatedU(i, j, k)) {
                        _MACVelocity.addU(i, j, k, _bodyForce.x * dt);
                    }
                }
            }
        }
    }

    if (fabs(_bodyForce.y) > 0.0) {
        for (int k = 0; k < _k_voxels; k++) {
            for (int j = 0; j < _j_voxels + 1; j++) {
                for (int i = 0; i < _i_voxels; i++) {
                    if (_isFaceBorderingMaterialV(i, j, k, M_FLUID) ||
                            _isFaceVelocityExtrapolatedV(i, j, k)) {
                        _MACVelocity.addV(i, j, k, _bodyForce.y * dt);
                    }
                }
            }
        }
    }

    if (fabs(_bodyForce.z) > 0.0) {
        for (int k = 0; k < _k_voxels + 1; k++) {
            for (int j = 0; j < _j_voxels; j++) {
                for (int i = 0; i < _i_voxels; i++) {
                    if (_isFaceBorderingMaterialW(i, j, k, M_FLUID) ||
                            _isFaceVelocityExtrapolatedW(i, j, k)) {
                        _MACVelocity.addW(i, j, k, _bodyForce.z * dt);
                    }
                }
            }
        }
    }
}

/********************************************************************************
    ADVECT FLUID VELOCITIES
********************************************************************************/

glm::vec3 FluidSimulation::_RK2(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = _MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 p1 = p0 + (float)dt*k2;

    return p1;
}

glm::vec3 FluidSimulation::_RK3(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = _MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 k3 = _MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.75*dt)*k2);
    glm::vec3 p1 = p0 + (float)(dt/9.0f)*(2.0f*k1 + 3.0f*k2 + 4.0f*k3);

    return p1;
}

glm::vec3 FluidSimulation::_RK4(glm::vec3 p0, glm::vec3 v0, double dt) {
    glm::vec3 k1 = v0;
    glm::vec3 k2 = _MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    glm::vec3 k3 = _MACVelocity.evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k2);
    glm::vec3 k4 = _MACVelocity.evaluateVelocityAtPosition(p0 + (float)dt*k3);
    glm::vec3 p1 = p0 + (float)(dt/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);

    return p1;
}

bool FluidSimulation::_isPointOnCellFace(glm::vec3 p, CellFace f) {
    double eps = 10e-6;

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
    assert(Grid3d::isGridIndexInRange(i, j, k, _i_voxels, _j_voxels, _k_voxels));
    
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

    assert(Grid3d::isGridIndexInRange(i, j, k, _i_voxels, _j_voxels, _k_voxels));

    std::vector<CellFace> faces;
    glm::vec3 normals[6] = { glm::vec3(-1.0, 0.0, 0.0), glm::vec3(1.0, 0.0, 0.0),
                             glm::vec3(0.0, -1.0, 0.0), glm::vec3(0.0, 1.0, 0.0), 
                             glm::vec3(0.0, 0.0, -1.0), glm::vec3(0.0, 0.0, 1.0) };
    GridIndex nc[26];
    _getNeighbourGridIndices26(i, j, k, nc);
    for (int idx = 0; idx < 26; idx++) {
        GridIndex c = nc[idx];
        if (Grid3d::isGridIndexInRange(c, _i_voxels, _j_voxels, _k_voxels) && 
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

    if (_isPointOnCellFace(*intersect, f)) {
        return true;
    }

    return false;
}

// Check if p lies on a cell face which borders a fluid cell and a solid cell. 
// if so, *f will store the face with normal pointing away from solid
bool FluidSimulation::_isPointOnSolidFluidBoundary(glm::vec3 p, CellFace *f) {
    int i, j, k;
    Grid3d::positionToGridIndex(p, _dx, &i, &j, &k);

    int U = 0; int V = 1; int W = 2;
    int side;
    int dir;
    CellFace faces[6];
    _getCellFaces(i, j, k, faces);
    for (int idx = 0; idx < 6; idx++) {
        if (_isPointOnCellFace(p, faces[idx])) {
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
    for (int idx = 0; idx < (int)allfaces.size(); idx++) {
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
    for (int idx = 0; idx < (int)faces.size(); idx++) {
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
    // p0 might lie right on a boundary face. In this case, the cell index of 
    // p0 may be calculated as a solid cell
    CellFace boundaryFace;
    bool isOnBoundary = _isPointOnSolidFluidBoundary(p0, &boundaryFace);
    if (isOnBoundary) {
        *normal = boundaryFace.normal;
        return p0;
    }

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

bool FluidSimulation::_integrateVelocity(glm::vec3 p0, glm::vec3 v0, double dt, glm::vec3 *p1) {
    *p1 = _RK4(p0, v0, dt);

    int ni, nj, nk;
    Grid3d::positionToGridIndex(*p1, _dx, &ni, &nj, &nk);

    if (_isCellSolid(ni, nj, nk)) {
        glm::vec3 collisionNormal;
        *p1 = _calculateSolidCellCollision(p0, *p1, &collisionNormal);
        
        // jog p1 back a bit from cell face
        *p1 = *p1 + (float)(0.01*_dx)*collisionNormal;

        Grid3d::positionToGridIndex(*p1, _dx, &ni, &nj, &nk);
        if (_isCellSolid(ni, nj, nk)) {
            *p1 = p0;
        }

        return false;
    }
    else {
        return true;
    }
}

void FluidSimulation::_backwardsAdvectVelocity(glm::vec3 p0, glm::vec3 v0, double dt, 
                                               glm::vec3 *p1, glm::vec3 *v1) {
    double timeleft = dt;
    while (timeleft > 0.0) {
        double timestep = _maxAdvectionDistanceFactor * _dx / glm::length(v0);
        timestep = (double)fminf((float)timeleft, (float)timestep);
        bool isOutsideBoundary = _integrateVelocity(p0, v0, -timestep, p1);
        *v1 = _MACVelocity.evaluateVelocityAtPosition(*p1);
        if (!isOutsideBoundary) {
            break;
        }

        p0 = *p1; v0 = *v1;
        timeleft -= timestep;
    }
}

void FluidSimulation::_advectVelocityFieldU(double dt) {
    glm::vec3 p0, p1, v0, v1;
    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels + 1; i++) {
                if (_isFaceBorderingMaterialU(i, j, k, M_FLUID)) {
                    p0 = _MACVelocity.velocityIndexToPositionU(i, j, k);
                    v0 = _MACVelocity.evaluateVelocityAtFaceCenterU(i, j, k);
                    _backwardsAdvectVelocity(p0, v0, dt, &p0, &v1);
                    _MACVelocity.setTempU(i, j, k, v1.x);
                }
            }
        }
    }
}

void FluidSimulation::_advectVelocityFieldV(double dt) {
    glm::vec3 p0, p1, v0, v1;
    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels + 1; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (_isFaceBorderingMaterialV(i, j, k, M_FLUID)) {
                    p0 = _MACVelocity.velocityIndexToPositionV(i, j, k);
                    v0 = _MACVelocity.evaluateVelocityAtFaceCenterV(i, j, k);
                    _backwardsAdvectVelocity(p0, v0, dt, &p0, &v1);
                    _MACVelocity.setTempV(i, j, k, v1.y);
                }
            }
        }
    }
}

void FluidSimulation::_advectVelocityFieldW(double dt) {
    glm::vec3 p0, p1, v0, v1;
    for (int k = 0; k < _k_voxels + 1; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (_isFaceBorderingMaterialW(i, j, k, M_FLUID)) {
                    p0 = _MACVelocity.velocityIndexToPositionW(i, j, k);
                    v0 = _MACVelocity.evaluateVelocityAtFaceCenterW(i, j, k);
                    _backwardsAdvectVelocity(p0, v0, dt, &p0, &v1);
                    _MACVelocity.setTempW(i, j, k, v1.z);
                }
            }
        }
    }
}

void FluidSimulation::_advectVelocityField(double dt) {
    _MACVelocity.resetTemporaryVelocityField();
    
    std::thread t1 = std::thread(&FluidSimulation::_advectVelocityFieldU, this, dt);
    std::thread t2 = std::thread(&FluidSimulation::_advectVelocityFieldV, this, dt);
    std::thread t3 = std::thread(&FluidSimulation::_advectVelocityFieldW, this, dt);

    t1.join();
    t2.join();
    t3.join();

    _MACVelocity.commitTemporaryVelocityFieldValues();
}

/********************************************************************************
    UPDATE PRESSURE GRID
********************************************************************************/

double FluidSimulation::_calculateNegativeDivergenceVector(VectorCoefficients &b) {
    double scale = 1.0 / _dx;

    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx++) {
        int i = _fluidCellIndices[idx].i;
        int j = _fluidCellIndices[idx].j;
        int k = _fluidCellIndices[idx].k;

        double value = -scale * (_MACVelocity.U(i + 1, j, k) - _MACVelocity.U(i, j, k) +
                                 _MACVelocity.V(i, j + 1, k) - _MACVelocity.V(i, j, k) +
                                 _MACVelocity.W(i, j, k + 1) - _MACVelocity.W(i, j, k));
        b.vector.set(i, j, k, value);
    }

    // solid cells are stationary right now
    double usolid = 0.0;
    double vsolid = 0.0;
    double wsolid = 0.0;
    double maxDivergence = 0.0;
    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx++) {
        int i = _fluidCellIndices[idx].i;
        int j = _fluidCellIndices[idx].j;
        int k = _fluidCellIndices[idx].k;

        if (_isCellSolid(i-1, j, k)) {
            double value = b.vector(i, j, k) - scale*(_MACVelocity.U(i, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }
        if (_isCellSolid(i+1, j, k)) {
            double value = b.vector(i, j, k) + scale*(_MACVelocity.U(i+1, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }

        if (_isCellSolid(i, j-1, k)) {
            double value = b.vector(i, j, k) - scale*(_MACVelocity.V(i, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }
        if (_isCellSolid(i, j+1, k)) {
            double value = b.vector(i, j, k) + scale*(_MACVelocity.V(i, j+1, k) - usolid);
            b.vector.set(i, j, k, value);
        }

        if (_isCellSolid(i, j, k-1)) {
            double value = b.vector(i, j, k) - scale*(_MACVelocity.W(i, j, k) - usolid);
            b.vector.set(i, j, k, value);
        }
        if (_isCellSolid(i, j, k+1)) {
            double value = b.vector(i, j, k) + scale*(_MACVelocity.W(i, j, k+1) - usolid);
            b.vector.set(i, j, k, value);
        }

        maxDivergence = fmax(maxDivergence, fabs(b.vector(i, j, k)));
    }

    return maxDivergence;
}

void FluidSimulation::_calculateMatrixCoefficients(MatrixCoefficients &A, double dt) {
    double scale = dt / (_density*_dx*_dx);

    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx ++) {
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
    double tau = 0.97;      // Tuning constant
    double sigma = 0.25;    // safety constant
    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx++) {
        int i = _fluidCellIndices[idx].i;
        int j = _fluidCellIndices[idx].j;
        int k = _fluidCellIndices[idx].k;

        double v1 = A.plusi(i - 1, j, k)*p.vector(i - 1, j, k);
        double v2 = A.plusj(i, j - 1, k)*p.vector(i, j - 1, k);
        double v3 = A.plusk(i, j, k - 1)*p.vector(i, j, k - 1);
        double v4 = p.vector(i - 1, j, k); v4 = v4*v4;
        double v5 = p.vector(i, j - 1, k); v5 = v5*v5;
        double v6 = p.vector(i, j, k - 1); v6 = v6*v6;

        double e = A.diag(i, j, k) - v1*v1 - v2*v2 - v3*v3 - 
            tau*(A.plusi(i - 1, j, k)*(A.plusj(i - 1, j, k) + A.plusk(i - 1, j, k))*v4 +
                 A.plusj(i, j - 1, k)*(A.plusi(i, j - 1, k) + A.plusk(i, j - 1, k))*v5 +
                 A.plusk(i, j, k - 1)*(A.plusi(i, j, k - 1) + A.plusj(i, j, k - 1))*v6);

        if (e < sigma*A.diag(i, j, k)) {
            e = A.diag(i, j, k);
        }

        if (fabs(e) > 10e-9) {
            p.vector.set(i, j, k, 1 / sqrt(e));
        }
    }
}

Eigen::VectorXd FluidSimulation::_VectorCoefficientsToEigenVectorXd(VectorCoefficients &v,
                                                                    std::vector<GridIndex> indices) {
    Eigen::VectorXd ev((int)indices.size());
    for (int idx = 0; idx < (int)indices.size(); idx++) {
        int i = indices[idx].i;
        int j = indices[idx].j;
        int k = indices[idx].k;

        ev(idx) = v.vector(i, j, k);
    }

    return ev;
}

void FluidSimulation::_EigenVectorXdToVectorCoefficients(Eigen::VectorXd v, 
                                                         VectorCoefficients &vc) {
    for (int idx = 0; idx < (int)v.size(); idx++) {
        GridIndex index = _VectorIndexToGridIndex(idx);
        vc.vector.set(index, v(idx));
    }
}

void FluidSimulation::_updateFluidGridIndexToEigenVectorXdIndexHashTable() {
    _GridIndexToEigenVectorXdIndex.fill(-1);

    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx++) {
        _GridIndexToEigenVectorXdIndex.set(_fluidCellIndices[idx], idx);
    }
}

GridIndex FluidSimulation::_VectorIndexToGridIndex(int i) {
    return _fluidCellIndices[i];
}

int FluidSimulation::_GridIndexToVectorIndex(int i, int j, int k) {
    GridIndex g(i, j, k);
    return _GridIndexToVectorIndex(g);
}

int FluidSimulation::_GridIndexToVectorIndex(GridIndex index) {
    int xdidx = _GridIndexToEigenVectorXdIndex(index);
    assert(xdidx != -1);
    return xdidx;
}

Eigen::VectorXd FluidSimulation::_applyPreconditioner(Eigen::VectorXd residualVector,
                                                      VectorCoefficients &p,
                                                      MatrixCoefficients &A) {
    VectorCoefficients r(_i_voxels, _j_voxels, _k_voxels);
    _EigenVectorXdToVectorCoefficients(residualVector, r);

    // Solve Aq = r
    VectorCoefficients q(_i_voxels, _j_voxels, _k_voxels);
    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx++) {
        GridIndex g = _fluidCellIndices[idx];
        int i = g.i;
        int j = g.j;
        int k = g.k;

        double t = r.vector(i, j, k) -
            A.plusi(i - 1, j, k)*p.vector(i - 1, j, k)*q.vector(i - 1, j, k) -
            A.plusj(i, j - 1, k)*p.vector(i, j - 1, k)*q.vector(i, j - 1, k) -
            A.plusk(i, j, k - 1)*p.vector(i, j, k - 1)*q.vector(i, j, k - 1);

        t = t*p.vector(i, j, k);
        q.vector.set(i, j, k, t);
    }

    // Solve transpose(A)*z = q
    VectorCoefficients z(_i_voxels, _j_voxels, _k_voxels);
    for (int idx = (int)_fluidCellIndices.size() - 1; idx >= 0; idx--) {
        GridIndex g = _fluidCellIndices[idx];
        int i = g.i;
        int j = g.j;
        int k = g.k;

        double precon = p.vector(i, j, k);
        double t = q.vector(i, j, k) -
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
                                                    MatrixCoefficients &A, double dt) {
    int size = (int)_fluidCellIndices.size();
    double scale = dt / (_density * _dx*_dx);

    std::vector<Eigen::Triplet<double>> matrixValues;
    matrixValues.reserve(size * (6 + 1));   // max number of non-zero entries 
                                            // (6 neighbours + diagonal) for each row
    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx++) {
        GridIndex g = _fluidCellIndices[idx];
        int i = g.i;
        int j = g.j;
        int k = g.k;

        int row = idx;
        int col = idx;
        double diag = _getNumFluidOrAirCellNeighbours(i, j, k)*scale;
        matrixValues.push_back(Eigen::Triplet<double>(col, row, diag));

        if (_isCellFluid(i-1, j, k)) {
            col = _GridIndexToVectorIndex(i-1, j, k);
            double coef = A.plusi(i-1, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
        if (_isCellFluid(i+1, j, k)) {
            col = _GridIndexToVectorIndex(i+1, j, k);
            double coef = A.plusi(i, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }

        if (_isCellFluid(i, j-1, k)) {
            col = _GridIndexToVectorIndex(i, j-1, k);
            double coef = A.plusj(i, j-1, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
        if (_isCellFluid(i, j+1, k)) {
            col = _GridIndexToVectorIndex(i, j+1, k);
            double coef = A.plusj(i, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }

        if (_isCellFluid(i, j, k-1)) {
            col = _GridIndexToVectorIndex(i, j, k-1);
            double coef = A.plusk(i, j, k-1);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
        if (_isCellFluid(i, j, k+1)) {
            col = _GridIndexToVectorIndex(i, j, k+1);
            double coef = A.plusk(i, j, k);
            matrixValues.push_back(Eigen::Triplet<double>(col, row, coef));
        }
    }

    Eigen::SparseMatrix<double> m(size, size);
    m.setFromTriplets(matrixValues.begin(), matrixValues.end());

    return m;
}

// Solve (A*p = b) with Eigen's conjugate gradient method
Eigen::VectorXd FluidSimulation::_solvePressureSystemWithEigen(MatrixCoefficients &A,
                                                               VectorCoefficients &b,
                                                               VectorCoefficients &precon,
                                                               double dt) {
    int size = (int)_fluidCellIndices.size();
    Eigen::VectorXd pressureVector(size); pressureVector.setZero();
    Eigen::VectorXd bVector = _VectorCoefficientsToEigenVectorXd(b, _fluidCellIndices);
    Eigen::VectorXd residualVector(bVector);

    if (fabs(residualVector.maxCoeff()) < _pressureSolveTolerance) {
        std::cout << "\tCG Iterations: " << 0 << std::endl;
        return pressureVector;
    }

    Eigen::SparseMatrix<double> aMatrix = _MatrixCoefficientsToEigenSparseMatrix(A, dt);

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double> > cg;
    cg.setMaxIterations(_maxPressureSolveIterations); 
    cg.setTolerance(_pressureSolveTolerance);

    cg.compute(aMatrix);
    pressureVector = cg.solve(bVector);
    std::cout << "\tCG Iterations:     " << cg.iterations() << std::endl;
    std::cout << "\testimated error: " << cg.error() << std::endl;

    return pressureVector;
}

// Solve (A*p = b) with Modified Incomplete Cholesky Conjugate Gradient menthod
// (MICCG(0))
Eigen::VectorXd FluidSimulation::_solvePressureSystem(MatrixCoefficients &A,
                                                      VectorCoefficients &b,
                                                      VectorCoefficients &precon,
                                                      double dt) {

    int size = (int)_fluidCellIndices.size();
    double tol = _pressureSolveTolerance;

    Eigen::VectorXd pressureVector(size); pressureVector.setZero();
    Eigen::VectorXd bVector = _VectorCoefficientsToEigenVectorXd(b, _fluidCellIndices);
    Eigen::VectorXd residualVector(bVector);

    if (fabs(residualVector.maxCoeff()) < tol) {
        return pressureVector;
    }

    Eigen::SparseMatrix<double> aMatrix = _MatrixCoefficientsToEigenSparseMatrix(A, dt);
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

void FluidSimulation::_resetMatrixCoefficients() {
    _matrixA.diag.fill(0.0);
    _matrixA.plusi.fill(0.0);
    _matrixA.plusj.fill(0.0);
    _matrixA.plusk.fill(0.0);
}

void FluidSimulation::_resetPreconditioner() {
    _preconditioner.vector.fill(0.0);
}

void FluidSimulation::_updatePressureGrid(double dt) {
    _pressureGrid.fill(0.0);

    VectorCoefficients b(_i_voxels, _j_voxels, _k_voxels);
    double maxDivergence = _calculateNegativeDivergenceVector(b);
    if (maxDivergence < _pressureSolveTolerance) {
        // all pressure values are near 0.0
        return;
    }

    _resetMatrixCoefficients();
    _resetPreconditioner();

    _calculateMatrixCoefficients(_matrixA, dt);
    _calculatePreconditionerVector(_preconditioner, _matrixA);
    _updateFluidGridIndexToEigenVectorXdIndexHashTable();
    Eigen::VectorXd pressures = _solvePressureSystem(_matrixA, b, _preconditioner, dt);
    
    for (int idx = 0; idx < (int)_fluidCellIndices.size(); idx++) {
        GridIndex index = _VectorIndexToGridIndex(idx);
        _pressureGrid.set(index, pressures(idx));
    }
}

/********************************************************************************
    UPDATE DIFFUSE MATERIAL PARTICLES
********************************************************************************/

void FluidSimulation::_sortMarkerParticlePositions(std::vector<glm::vec3> &surface, 
                                                   std::vector<glm::vec3> &inside) {
    glm::vec3 p;
    double width = _diffuseSurfaceNarrowBandSize * _dx;
    for (int i = 0; i < (int)_markerParticles.size(); i++) {
        p = _markerParticles[i].position;
        if (_levelset.getDistance(p) < width) {
            surface.push_back(p);
        } else if (_levelset.isPointInInsideCell(p)) {
            inside.push_back(p);
        }
    }
}

double FluidSimulation::_getWavecrestPotential(glm::vec3 p, glm::vec3 *v) {

    *v = _MACVelocity.evaluateVelocityAtPositionLinear(p);
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

double FluidSimulation::_getEnergyPotential(glm::vec3 p, glm::vec3 velocity) {
    double e = 0.5*glm::dot(velocity, velocity);
    e = fmax(e, _minParticleEnergy);
    e = fmin(e, _maxParticleEnergy);

    return (e - _minParticleEnergy) / (_maxParticleEnergy - _minParticleEnergy);
}

void FluidSimulation::_getDiffuseParticleEmitters(std::vector<DiffuseParticleEmitter> &emitters) {

    std::vector<glm::vec3> surfaceParticles;
    std::vector<glm::vec3> insideParticles;
    _sortMarkerParticlePositions(surfaceParticles, insideParticles);

    _levelset.calculateSurfaceCurvature();

    _turbulenceField.calculateTurbulenceField(&_MACVelocity, _fluidCellIndices);

    glm::vec3 p;
    for (int i = 0; i < (int)surfaceParticles.size(); i++) {
        p = surfaceParticles[i];

        glm::vec3 velocity;
        double Iwc = _getWavecrestPotential(p, &velocity);
        double It = _getTurbulencePotential(p, _turbulenceField);

        if (Iwc > 0.0 || It > 0.0) {
            double Ie = _getEnergyPotential(p, velocity);
            if (Ie > 0.0) {
                emitters.push_back(DiffuseParticleEmitter(p, velocity, Ie, Iwc, It));
            }
        }
    }

    for (int i = 0; i < (int)insideParticles.size(); i++) {
        p = insideParticles[i];
        double It = _getTurbulencePotential(p, _turbulenceField);

        if (It > 0.0) {
            glm::vec3 velocity = _MACVelocity.evaluateVelocityAtPositionLinear(p);
            double Ie = _getEnergyPotential(p, velocity);
            if (Ie > 0.0) {
                emitters.push_back(DiffuseParticleEmitter(p, velocity, Ie, 0.0, It));
            }
        }
    }
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

    glm::vec3 axis = glm::normalize(emitter.velocity);

    double eps = 10e-6;
    glm::vec3 e1;
    if (fabs(axis.x) - 1.0 < eps && fabs(axis.y) < eps && fabs(axis.z) < eps) {
        e1 = glm::normalize(glm::cross(axis, glm::vec3(1.0, 0.0, 0.0)));
    } else {
        e1 = glm::normalize(glm::cross(axis, glm::vec3(0.0, 1.0, 0.0)));
    }

    e1 = e1*(float)_markerParticleRadius;
    glm::vec3 e2 = glm::normalize(glm::cross(axis, e1)) * (float)_markerParticleRadius;

    float Xr, Xt, Xh, r, theta, h, sinval, cosval, lifetime;
    glm::vec3 p, v;
    GridIndex g;
    for (int i = 0; i < n; i++) {
        Xr = (float)(rand()) / (float)RAND_MAX;
        Xt = (float)(rand()) / (float)RAND_MAX;
        Xh = (float)(rand()) / (float)RAND_MAX;

        r = _markerParticleRadius*sqrt(Xr);
        theta = Xt*2.0f*3.141592653;
        h = Xh*glm::length((float)dt*emitter.velocity);
        sinval = sin(theta);
        cosval = cos(theta);

        p = emitter.position + r*cosval*e1 + r*sinval*e2 + h*axis;
        g = Grid3d::positionToGridIndex(p, _dx);
        if (!Grid3d::isGridIndexInRange(g, _i_voxels, _j_voxels, _k_voxels) || 
                _isCellSolid(g)) {
            continue;
        }

        v = emitter.velocity + r*cosval*e1 + r*sinval*e2;
        lifetime = (float)(emitter.energyPotential*_maxDiffuseParticleLifetime);
        _diffuseParticles.push_back(DiffuseParticle(p, v, lifetime));
    }
}

void FluidSimulation::_emitDiffuseParticles(std::vector<DiffuseParticleEmitter> &emitters,
                                            double dt) {
    for (int i = 0; i < (int)emitters.size(); i++) {
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

    return type;
}

void FluidSimulation::_updateDiffuseParticleTypesAndVelocities() {
    DiffuseParticle dp;
    double dist;
    int type;
    for (int i = 0; i < (int)_diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        type = _getDiffuseParticleType(dp);
        _diffuseParticles[i].type = type;

        // Foam particles don't need to calculate their own velocity,
        // but spray and bubble particles do, so reinitialize velocity
        // when transitioning from foam type.
        if (type != DP_FOAM && dp.type == DP_FOAM) {
            glm::vec3 v = _MACVelocity.evaluateVelocityAtPositionLinear(dp.position);
            _diffuseParticles[i].velocity = v;
        }
    }
}

void FluidSimulation::_updateDiffuseParticleLifetimes(double dt) {
    std::vector<DiffuseParticle> livingParticles;
    livingParticles.reserve(_diffuseParticles.size());

    DiffuseParticle dp;
    for (int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        if (dp.type == DP_FOAM) {
            _diffuseParticles[i].lifetime = dp.lifetime - dt;
        }

        if (_diffuseParticles[i].lifetime > 0.0) {
            livingParticles.push_back(_diffuseParticles[i]);
        }
    }
    _diffuseParticles = livingParticles;
}

void FluidSimulation::_getNextBubbleDiffuseParticle(DiffuseParticle &dp,
                                                    DiffuseParticle &nextdp,
                                                    double dt) {
    glm::vec3 vmac = _MACVelocity.evaluateVelocityAtPositionLinear(dp.position);
    glm::vec3 vbub = dp.velocity;
    glm::vec3 bouyancyVelocity = (float)-_bubbleBouyancyCoefficient*_bodyForce;
    glm::vec3 dragVelocity = (float)_bubbleDragCoefficient*(vmac - vbub) / (float)dt;

    nextdp.velocity += (float)dt*(bouyancyVelocity + dragVelocity);
    nextdp.position += nextdp.velocity*(float)dt;
}

void FluidSimulation::_getNextSprayDiffuseParticle(DiffuseParticle &dp,
                                                   DiffuseParticle &nextdp,
                                                   double dt) {
    nextdp.velocity += _bodyForce*(float)dt;
    nextdp.position += nextdp.velocity*(float)dt;
}

void FluidSimulation::_getNextFoamDiffuseParticle(DiffuseParticle &dp,
                                                  DiffuseParticle &nextdp,
                                                  double dt) {
    glm::vec3 v0 = _MACVelocity.evaluateVelocityAtPosition(dp.position);
    nextdp.velocity = dp.velocity;
    nextdp.position = _RK2(dp.position, v0, dt);
}

void FluidSimulation::_advanceDiffuseParticles(double dt) {
    DiffuseParticle dp, nextdp;
    GridIndex g;
    for (int i = 0; i < _diffuseParticles.size(); i++) {
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
            nextdp.position = coll + (float)(0.1*_dx)*norm;
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
    _updateDiffuseParticleTypesAndVelocities();
    _updateDiffuseParticleLifetimes(dt);
    _advanceDiffuseParticles(dt);
}

/********************************************************************************
    ADVANCE MARKER PARTICLES
********************************************************************************/

void FluidSimulation::_advanceRangeOfMarkerParticles(int startIdx, int endIdx, double dt) {
    assert(startIdx <= endIdx);

    MarkerParticle mp;
    glm::vec3 p;
    glm::vec3 vi;
    for (int idx = startIdx; idx <= endIdx; idx++) {
        mp = _markerParticles[idx];

        vi = _MACVelocity.evaluateVelocityAtPosition(mp.position);
        p = _RK4(mp.position, vi, dt);

        if (!Grid3d::isPositionInGrid(p.x, p.y, p.z, _dx, _i_voxels, _j_voxels, _k_voxels)) {
            continue;
        }

        int i, j, k;
        Grid3d::positionToGridIndex(p, _dx, &i, &j, &k);

        if (_isCellSolid(i, j, k)) {
            glm::vec3 norm;
            glm::vec3 coll = _calculateSolidCellCollision(mp.position, p, &norm);

            // jog p back a bit from cell face
            p = coll + (float)(0.1*_dx)*norm;
        }

        Grid3d::positionToGridIndex(p, _dx, &i, &j, &k);
        if (!_isCellSolid(i, j, k)) {
            _markerParticles[idx].position = p;
            _markerParticles[idx].index = GridIndex(i, j, k);
        }
    }
    
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
}

void FluidSimulation::_applyPressureToVelocityField(double dt) {
    _MACVelocity.resetTemporaryVelocityField();

    double usolid = 0.0;   // solids are stationary
    double scale = dt / (_density * _dx);
    double invscale = 1.0 / scale;

    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels + 1; i++) {
                if (_isFaceBorderingMaterialU(i, j, k, M_FLUID)) {
                    int ci = i - 1; int cj = j; int ck = k;

                    double p0, p1;
                    if (!_isCellSolid(ci, cj, ck) && !_isCellSolid(ci + 1, cj, ck)) {
                        p0 = _pressureGrid(ci, cj, ck);
                        p1 = _pressureGrid(ci + 1, cj, ck);
                    } else if (_isCellSolid(ci, cj, ck)) {
                        p0 = _pressureGrid(ci + 1, cj, ck) - 
                             invscale*(_MACVelocity.U(i, j, k) - usolid);
                        p1 = _pressureGrid(ci + 1, cj, ck);
                    } else {
                        p0 = _pressureGrid(ci, cj, ck);
                        p1 = _pressureGrid(ci, cj, ck) +
                             invscale*(_MACVelocity.U(i, j, k) - usolid);
                    }

                    double unext = _MACVelocity.U(i, j, k) - scale*(p1 - p0);
                    _MACVelocity.setTempU(i, j, k, unext);
                }
            }
        }
    }

    for (int k = 0; k < _k_voxels; k++) {
        for (int j = 0; j < _j_voxels + 1; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (_isFaceBorderingMaterialV(i, j, k, M_FLUID)) {
                    int ci = i; int cj = j - 1; int ck = k;

                    double p0, p1;
                    if (!_isCellSolid(ci, cj, ck) && !_isCellSolid(ci, cj + 1, ck)) {
                        p0 = _pressureGrid(ci, cj, ck);
                        p1 = _pressureGrid(ci, cj + 1, ck);
                    }
                    else if (_isCellSolid(ci, cj, ck)) {
                        p0 = _pressureGrid(ci, cj + 1, ck) -
                            invscale*(_MACVelocity.V(i, j, k) - usolid);
                        p1 = _pressureGrid(ci, cj + 1, ck);
                    }
                    else {
                        p0 = _pressureGrid(ci, cj, ck);
                        p1 = _pressureGrid(ci, cj, ck) +
                            invscale*(_MACVelocity.V(i, j, k) - usolid);
                    }

                    double vnext = _MACVelocity.V(i, j, k) - scale*(p1 - p0);
                    _MACVelocity.setTempV(i, j, k, vnext);
                }
            }
        }
    }

    for (int k = 0; k < _k_voxels + 1; k++) {
        for (int j = 0; j < _j_voxels; j++) {
            for (int i = 0; i < _i_voxels; i++) {
                if (_isFaceBorderingMaterialW(i, j, k, M_FLUID)) {
                    int ci = i; int cj = j; int ck = k - 1;

                    double p0, p1;
                    if (!_isCellSolid(ci, cj, ck) && !_isCellSolid(ci, cj, ck + 1)) {
                        p0 = _pressureGrid(ci, cj, ck);
                        p1 = _pressureGrid(ci, cj, ck + 1);
                    }
                    else if (_isCellSolid(ci, cj, ck)) {
                        p0 = _pressureGrid(ci, cj, ck + 1) -
                             invscale*(_MACVelocity.W(i, j, k) - usolid);
                        p1 = _pressureGrid(ci, cj, ck + 1);
                    }
                    else {
                        p0 = _pressureGrid(ci, cj, ck);
                        p1 = _pressureGrid(ci, cj, ck) +
                             invscale*(_MACVelocity.W(i, j, k) - usolid);
                    }

                    double wnext = _MACVelocity.W(i, j, k) - scale*(p1 - p0);
                    _MACVelocity.setTempW(i, j, k, wnext);
                }
            }
        }
    }

    _MACVelocity.commitTemporaryVelocityFieldValues();
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
    _extrapolateFluidVelocities();
    timer5.stop();

    _logfile.log("Extrapolate Fluid Velocities:\t", timer5.getTime(), 4);

    timer6.start();
    _applyBodyForcesToVelocityField(dt);
    timer6.stop();

    _logfile.log("Apply Body Forces:           \t", timer6.getTime(), 4);

    timer7.start();
    _advectVelocityField(dt);
    timer7.stop();

    _logfile.log("Advect Velocity Field:       \t", timer7.getTime(), 4);

    timer8.start();
    _updatePressureGrid(dt);
    timer8.stop();

    _logfile.log("Update Pressure Grid:        \t", timer8.getTime(), 4);

    timer9.start();
    _applyPressureToVelocityField(dt);
    timer9.stop();

    _logfile.log("Apply Pressure:              \t", timer9.getTime(), 4);

    timer10.start();
    _updateDiffuseMaterial(dt);
    timer10.stop();

    _logfile.log("Update Diffuse Material:     \t", timer10.getTime(), 4);

    timer11.start();
    _advanceMarkerParticles(dt);
    timer11.stop();

    _logfile.log("Advance Marker Particles:    \t", timer11.getTime(), 4);

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

    _logfile.log("---Percentage Breakdown---", "");
    _logfile.log("Update Fluid Cells:          \t", p2, 3);
    _logfile.log("Reconstruct Fluid Surface    \t", p3, 3);
    _logfile.log("Update Level Set:            \t", p4, 3);
    _logfile.log("Extrapolate Fluid Velocities:\t", p5, 3);
    _logfile.log("Apply Body Forces:           \t", p6, 3);
    _logfile.log("Advect Velocity Field:       \t", p7, 3);
    _logfile.log("Update Pressure Grid:        \t", p8, 3);
    _logfile.log("Apply Pressure:              \t", p9, 3);
    _logfile.log("Update Diffuse Material:     \t", p10, 3);
    _logfile.log("Advance Marker Particles:    \t", p11, 3);
    _logfile.newline();

    _logfile.log("Simulation time: ", _simulationTime, 3);
    _logfile.log("Real time: ", _realTime, 2);
    _logfile.newline();
    _logfile.write();
}

double FluidSimulation::_calculateNextTimeStep() {
    double maxu = _MACVelocity.evaluateMaximumVelocityMagnitude();
    double timeStep = _CFLConditionNumber*_dx / maxu;

    timeStep = (double)fmaxf((float)_minTimeStep, (float)timeStep);
    timeStep = (double)fminf((float)_maxTimeStep, (float)timeStep);

    return timeStep;
}

void FluidSimulation::update(double dt) {
    if (!_isSimulationRunning || !_isSimulationInitialized) {
        return;
    }
    _isCurrentFrameFinished = false;

    saveState();

    _currentTimeStep = 0;
    double timeleft = dt;
    while (timeleft > 0.0) {
        double timestep = _calculateNextTimeStep();
        if (timeleft - timestep < 0.0) {
            timestep = timeleft;
        }
        timeleft -= timestep;
        _stepFluid(timestep);

        _currentTimeStep++;
    }

    _writeSurfaceMeshToFile();
    _currentFrame++;

    _isCurrentFrameFinished = true;
}

void FluidSimulation::draw() {

}