#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>
#include <thread>
#include <unordered_map>
#include <assert.h>

#include <Eigen\Core>
#include <Eigen\SparseCore>
#include <Eigen\IterativeLinearSolvers>
#include <gl\glew.h>
#include <SDL_opengl.h>
#include <gl\glu.h>

#include "stopwatch.h"
#include "macvelocityfield.h"
#include "array3d.h"
#include "grid3d.h"
#include "surfacefield.h"
#include "levelsetfield.h"
#include "implicitsurfacescalarfield.h"
#include "polygonizer3d.h"
#include "triangleMesh.h"
#include "logfile.h"
#include "collision.h"
#include "aabb.h"
#include "levelset.h"
#include "fluidsimulationsavestate.h"
#include "fluidsource.h"
#include "sphericalfluidsource.h"
#include "cuboidfluidsource.h"
#include "turbulencefield.h"
#include "glm/glm.hpp"

class FluidSimulation
{
public:
    FluidSimulation();
    FluidSimulation(int x_voxels, int y_voxels, int z_voxels, double cell_size);
    FluidSimulation(FluidSimulationSaveState &state);
    ~FluidSimulation();

    void update(double dt);
    void draw();

    void run();
    void pause();
        
    double getCellSize() { return _dx; }
    void getGridDimensions(int *i, int *j, int *k) { *i = _i_voxels; *j = _j_voxels; *k = _k_voxels; }
    void getSimulationDimensions(double *w, double *h, double *d) { *w = (double)_i_voxels*_dx;
                                                                    *h = (double)_j_voxels*_dx;
                                                                    *d = (double)_k_voxels*_dx; }
    double getSimulationWidth() {  return (double)_i_voxels*_dx; }
    double getSimulationHeight() { return (double)_j_voxels*_dx; }
    double getSimulationDepth() {  return (double)_k_voxels*_dx; }

    int getMaterial(int i, int j, int k) { 
        return _materialGrid(i, j, k); 
    }

    std::vector<glm::vec3> getMarkerParticles();
    std::vector<glm::vec3> getMarkerParticles(int skip);
    Array3d<int> getLayerGrid() { return _layerGrid; }

    void addBodyForce(double fx, double fy, double fz) { addBodyForce(glm::vec3(fx, fy, fz)); }
    void addBodyForce(glm::vec3 f);
    void setBodyForce(double fx, double fy, double fz) { setBodyForce(glm::vec3(fx, fy, fz)); }
    void setBodyForce(glm::vec3 f);

    double getDensity() { return _density; }
    void setDensity(double p) { assert(p > 0); _density = p; }

    void setMarkerParticleScale(double s) { _markerParticleScale = s; }

    MACVelocityField* getVelocityField() { return &_MACVelocity; }
    LevelSet* getLevelSet() { return &_levelset; };

    void addImplicitFluidPoint(double x, double y, double z, double r) {
        addImplicitFluidPoint(glm::vec3(x, y, z), r);
    }
    void addImplicitFluidPoint(glm::vec3 p, double radius);
    void addFluidCuboid(double x, double y, double z, double w, double h, double d) {
        addFluidCuboid(glm::vec3(x, y, z), w, h, d);
    }
    void addFluidCuboid(glm::vec3 p, double width, double height, double depth);
    void addFluidCuboid(glm::vec3 p1, glm::vec3 p2);

    bool addFluidMesh(std::string OBJFilename) {
        return addFluidMesh(OBJFilename, glm::vec3(0.0, 0.0, 0.0), 1.0);
    }
    bool addFluidMesh(std::string OBJFilename, glm::vec3 offset) {
        return addFluidMesh(OBJFilename, offset, 1.0);
    }
    bool addFluidMesh(std::string OBJFilename, double scale) {
        return addFluidMesh(OBJFilename, glm::vec3(0.0, 0.0, 0.0), scale);
    }
    bool addFluidMesh(std::string OBJFilename, glm::vec3 offset, double scale);

    SphericalFluidSource *addSphericalFluidSource(glm::vec3 pos, double r);
    SphericalFluidSource *addSphericalFluidSource(glm::vec3 pos, double r, 
                                                  glm::vec3 velocity);
    CuboidFluidSource *addCuboidFluidSource(AABB bbox);
    CuboidFluidSource *addCuboidFluidSource(AABB bbox, glm::vec3 velocity);

    void addSolidCell(int i, int j, int k);
    void addSolidCells(std::vector<glm::vec3> indices);
    void removeSolidCell(int i, int j, int k);
    void removeSolidCells(std::vector<glm::vec3> indices);
    std::vector<glm::vec3> getSolidCells();
    std::vector<glm::vec3> getSolidCellPositions();
    TriangleMesh* getFluidSurfaceTriangles() { return &_surfaceMesh; }

    bool isCurrentFrameFinished() { return _isCurrentFrameFinished; }

    void saveState();
    void saveState(std::string filename);
    int getCurrentFrame();
    
private:
    struct MarkerParticle {
        glm::vec3 position = glm::vec3(0.0, 0.0, 0.0);
        GridIndex index = GridIndex(0, 0, 0);

        MarkerParticle() : position(glm::vec3(0.0, 0.0, 0.0)), index(GridIndex(0, 0, 0)) {}

        MarkerParticle(glm::vec3 p, int ii, int jj, int kk) : position(p),
                                                              index(GridIndex(ii, jj, kk)) {}

        MarkerParticle(double x, double y, double z, int ii, int jj, int kk) : 
                        position(glm::vec3(x, y, z)),
                        index(GridIndex(ii, jj, kk)) {}
    };

    struct DiffuseParticleEmitter {
        glm::vec3 position;
        glm::vec3 velocity;
        double energyPotential;
        double wavecrestPotential;
        double turbulencePotential;

        DiffuseParticleEmitter() : position(0.0, 0.0, 0.0),
                                   velocity(0.0, 0.0, 0.0),
                                   energyPotential(0.0),
                                   wavecrestPotential(0.0),
                                   turbulencePotential(0.0) {}

        DiffuseParticleEmitter(glm::vec3 p, glm::vec3 v, 
                               double e, double wc, double t) : 
                                   position(p),
                                   velocity(v),
                                   energyPotential(e),
                                   wavecrestPotential(wc),
                                   turbulencePotential(t) {}
    };

    struct CellFace {
        glm::vec3 normal;
        double minx, maxx;
        double miny, maxy;
        double minz, maxz;

        CellFace() : normal(glm::vec3(0.0, 0.0, 0.0)),
                     minx(0.0), maxx(0.0),
                     miny(0.0), maxy(0.0),
                     minz(0.0), maxz(0.0) {}

        CellFace(glm::vec3 n, glm::vec3 minp, glm::vec3 maxp) :
                                    normal(n), minx(minp.x), maxx(maxp.x),
                                               miny(minp.y), maxy(maxp.y),
                                               minz(minp.z), maxz(maxp.z) {}
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

    struct VectorCoefficients {
        Array3d<double> vector;
        int width, height, depth;

        VectorCoefficients() : width(0), height(0), depth(0) {}
        VectorCoefficients(int i, int j, int k) : vector(Array3d<double>(i, j, k, 0.0)),
                                                width(i), height(j), depth(k) {}
    };

    int M_AIR = 0;
    int M_FLUID = 1;
    int M_SOLID = 2;
    int T_INFLOW = 0;
    int T_OUTFLOW = 1;

    // Initialization before running simulation
    void _initializeSimulation();
    void _initializeSolidCells();
    void _initializeFluidMaterial();
    void _addMarkerParticlesToCell(GridIndex g);
    void _initializeSimulationFromSaveState(FluidSimulationSaveState &state);
    void _initializeMarkerParticlesFromSaveState(FluidSimulationSaveState &state);
    void _initializeFluidMaterialParticlesFromSaveState();
    void _initializeSolidCellsFromSaveState(FluidSimulationSaveState &state);
    void _initializeMACGridFromSaveState(FluidSimulationSaveState &state);

    // Simulation step
    double _calculateNextTimeStep();
    void _stepFluid(double dt);

    // Find fluid cells. Fluid cells must contain at
    // least 1 marker particle
    void _updateFluidCells();
    void _updateFluidSources();
    void _updateFluidSource(FluidSource *source);
    void _addNewFluidCells(std::vector<GridIndex> &cells, glm::vec3 velocity);
    void _setVelocitiesForNewFluidCell(GridIndex g, glm::vec3 v);
    void _removeMarkerParticlesFromCells(std::vector<GridIndex> &cells);
    inline bool _isIndexInList(GridIndex g, 
                               std::vector<GridIndex> &list) {
        GridIndex c;
        for (int idx = 0; idx < (int)list.size(); idx++) {
            c = list[idx];
            if (g.i == c.i && g.j == c.j && g.k == c.k) {
                return true;
            }
        }
        return false;
    }

    // Convert marker particles to fluid surface
    void _reconstructFluidSurface();
    void _writeSurfaceMeshToFile();

    // Update level set surface
    void _updateLevelSetSignedDistance();

    // Extrapolate fluid velocities into surrounding air and solids so
    // that velocities can be computed when marker particles move to cells
    // outside of current fluid region
    void _extrapolateFluidVelocities();
    void _resetExtrapolatedFluidVelocities();
    int _updateExtrapolationLayers();
    void _updateExtrapolationLayer(int layerIndex);
    void _extrapolateVelocitiesForLayerIndex(int layerIndex);
    double _getExtrapolatedVelocityForFaceU(int i, int j, int k, int layerIndex);
    double _getExtrapolatedVelocityForFaceV(int i, int j, int k, int layerIndex);
    double _getExtrapolatedVelocityForFaceW(int i, int j, int k, int layerIndex);
    glm::vec3 _getVelocityAtNearestPointOnFluidSurface(glm::vec3 p);


    // Add gravity to fluid velocities and extrapolated velocities
    void _applyBodyForcesToVelocityField(double dt);

    // Advect fluid velocities, but not extrapolated velocities
    void _advectVelocityField(double dt);
    void _advectVelocityFieldU(double dt);
    void _advectVelocityFieldV(double dt);
    void _advectVelocityFieldW(double dt);
    void _backwardsAdvectVelocity(glm::vec3 p0, glm::vec3 v0, double dt, glm::vec3 *p1, glm::vec3 *v1);
    bool _integrateVelocity(glm::vec3 p0, glm::vec3 v0, double dt, glm::vec3 *p1);

    // Calculate pressure values to satisfy incompressibility condition
    void _updatePressureGrid(double dt);
    double _calculateNegativeDivergenceVector(VectorCoefficients &b);
    void _calculateMatrixCoefficients(MatrixCoefficients &A, double dt);
    void _calculatePreconditionerVector(VectorCoefficients &precon, MatrixCoefficients &A);
    Eigen::VectorXd _applyPreconditioner(Eigen::VectorXd r, 
                                         VectorCoefficients &precon,
                                         MatrixCoefficients &A);
    Eigen::VectorXd _solvePressureSystem(MatrixCoefficients &A, 
                                         VectorCoefficients &b, 
                                         VectorCoefficients &precon,
                                         double dt);
    Eigen::VectorXd _solvePressureSystemWithEigen(MatrixCoefficients &A,
                                                  VectorCoefficients &b,
                                                  VectorCoefficients &precon,
                                                  double dt);

    // Methods for seting up system of equations for the pressure update
    void _resetMatrixCoefficients();
    void _resetPreconditioner();
    void _EigenVectorXdToVectorCoefficients(Eigen::VectorXd v, VectorCoefficients &vc);
    Eigen::VectorXd _VectorCoefficientsToEigenVectorXd(VectorCoefficients &p,
        std::vector<GridIndex> indices);
    Eigen::SparseMatrix<double> _MatrixCoefficientsToEigenSparseMatrix(MatrixCoefficients &A,
        double dt);
    void _updateFluidGridIndexToEigenVectorXdIndexHashTable();
    int _GridIndexToVectorIndex(int i, int j, int k);
    int _GridIndexToVectorIndex(GridIndex index);
    GridIndex _VectorIndexToGridIndex(int index);
    int _getNumFluidOrAirCellNeighbours(int i, int j, int k);

    // Alter fluid velocities according to calculated pressures
    // to create a divercence free velocity field
    void _applyPressureToVelocityField(double dt);

    // Update diffuse material (spray, foam, bubbles)
    void _updateDiffuseMaterial(double dt);
    void _sortMarkerParticlePositions(std::vector<glm::vec3> &surface, 
                                      std::vector<glm::vec3> &inside);
    void _getDiffuseParticleEmitters(std::vector<DiffuseParticleEmitter> &emitters);
    double _getWavecrestPotential(glm::vec3 p, glm::vec3 *velocity);
    double _getTurbulencePotential(glm::vec3 p, TurbulenceField &tfield);
    double _getEnergyPotential(glm::vec3 p, glm::vec3 velocity);

    // Move marker particles through the velocity field
    void _advanceMarkerParticles(double dt);
    void _advanceRangeOfMarkerParticles(int startIdx, int endIdx, double dt);

    // Methods for finding collisions between marker particles and solid cell
    // boundaries. Also used for advecting fluid when particle enters a solid.
    std::vector<CellFace> _getNeighbourSolidCellFaces(int i, int j, int k);
    bool _isPointOnCellFace(glm::vec3 p, CellFace f);
    bool _isPointOnSolidFluidBoundary(glm::vec3 p, CellFace *f);
    CellFace _getCellFace(int i, int j, int k, glm::vec3 normal);
    void _getCellFaces(int i, int j, int k, CellFace[6]);
    bool _getVectorFaceIntersection(glm::vec3 p0, glm::vec3 normal, CellFace f, glm::vec3 *intersect);
    glm::vec3 _calculateSolidCellCollision(glm::vec3 p0, glm::vec3 p1, glm::vec3 *normal);
    std::vector<CellFace> _getSolidCellFaceCollisionCandidates(int i, int j, int k, glm::vec3 dir);
    bool _findFaceCollision(glm::vec3 p0, glm::vec3 p1, CellFace *face, glm::vec3 *intersection);
    void _getNeighbourGridIndices6(int i, int j, int k, GridIndex n[6]);
    void _getNeighbourGridIndices26(int i, int j, int k, GridIndex n[26]);

    // Runge-Kutta integrators used in advection and advancing marker particles
    glm::vec3 _RK2(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK3(glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _RK4(glm::vec3 p0, glm::vec3 v0, double dt);

    // misc bool functions for checking cell contents and borders
    inline bool _isCellAir(int i, int j, int k) { return _materialGrid(i, j, k) == M_AIR; }
    inline bool _isCellFluid(int i, int j, int k) { return _materialGrid(i, j, k) == M_FLUID; }
    inline bool _isCellSolid(int i, int j, int k) { return _materialGrid(i, j, k) == M_SOLID; }
    inline bool _isCellAir(GridIndex g) { return _materialGrid(g) == M_AIR; }
    inline bool _isCellFluid(GridIndex g) { return _materialGrid(g) == M_FLUID; }
    inline bool _isCellSolid(GridIndex g) { return _materialGrid(g) == M_SOLID; }
    

    inline bool _isFaceBorderingGridValueU(int i, int j, int k, int value, Array3d<int> &grid) {
        if (i == grid.width) { return grid(i - 1, j, k) == value; }
        else if (i > 0) { return grid(i, j, k) == value || grid(i - 1, j, k) == value; }
        else { return grid(i, j, k) == value; }
    }
    inline bool _isFaceBorderingGridValueV(int i, int j, int k, int value, Array3d<int> &grid) {
        if (j == grid.height) { return grid(i, j - 1, k) == value; }
        else if (j > 0) { return grid(i, j, k) == value || grid(i, j - 1, k) == value; }
        else { return grid(i, j, k) == value; }
    }
    inline bool _isFaceBorderingGridValueW(int i, int j, int k, int value, Array3d<int> &grid) {
        if (k == grid.depth) { return grid(i, j, k - 1) == value; }
        else if (k > 0) { return grid(i, j, k) == value || grid(i, j, k - 1) == value; }
        else { return grid(i, j, k) == value; }
    }

    inline bool _isFaceBorderingGridValueU(GridIndex g, int value, Array3d<int> &grid) {
        return _isFaceBorderingGridValueU(g.i, g.j, g.k, value, grid);
    }
    inline bool _isFaceBorderingGridValueV(GridIndex g, int value, Array3d<int> &grid) {
        return _isFaceBorderingGridValueV(g.i, g.j, g.k, value, grid);
    }
    inline bool _isFaceBorderingGridValueW(GridIndex g, int value, Array3d<int> &grid) {
        return _isFaceBorderingGridValueW(g.i, g.j, g.k, value, grid);
    }

    inline bool _isFaceBorderingMaterialU(int i, int j, int k, int mat) {
        return _isFaceBorderingGridValueU(i, j, k, mat, _materialGrid);
    }
    inline bool _isFaceBorderingMaterialV(int i, int j, int k, int mat) {
        return _isFaceBorderingGridValueV(i, j, k, mat, _materialGrid);
    }
    inline bool _isFaceBorderingMaterialW(int i, int j, int k, int mat) {
        return _isFaceBorderingGridValueW(i, j, k, mat, _materialGrid);
    }

    inline bool _isFaceBorderingLayerIndexU(int i, int j, int k, int layer) {
        return _isFaceBorderingGridValueU(i, j, k, layer, _layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexV(int i, int j, int k, int layer) {
        return _isFaceBorderingGridValueV(i, j, k, layer, _layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexW(int i, int j, int k, int layer) {
        return _isFaceBorderingGridValueW(i, j, k, layer, _layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexU(GridIndex g, int layer) {
        return _isFaceBorderingGridValueU(g, layer, _layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexV(GridIndex g, int layer) {
        return _isFaceBorderingGridValueV(g, layer, _layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexW(GridIndex g, int layer) {
        return _isFaceBorderingGridValueW(g, layer, _layerGrid);
    }

    inline bool _isFaceVelocityExtrapolatedU(int i, int j, int k) {
        if (i == _i_voxels) {  return _layerGrid(i - 1, j, k) >= 1.0; }
        else if (i > 0) { return _layerGrid(i, j, k) >= 1.0 || _layerGrid(i - 1, j, k) >= 1.0; }
        else { return _layerGrid(i, j, k) >= 1.0; }
    }

    inline bool _isFaceVelocityExtrapolatedV(int i, int j, int k) {
        if (j == _j_voxels) { return _layerGrid(i, j - 1, k) >= 1.0; }
        else if (j > 0) { return _layerGrid(i, j, k) >= 1.0 || _layerGrid(i, j - 1, k) >= 1.0; }
        else { return _layerGrid(i, j, k) >= 1.0; }
    }

    inline bool _isFaceVelocityExtrapolatedW(int i, int j, int k) {
        if (k == _k_voxels) { return _layerGrid(i, j, k - 1) >= 1.0; }
        else if (k > 0) { return _layerGrid(i, j, k) >= 1.0 || _layerGrid(i, j, k - 1) >= 1.0; }
        else { return _layerGrid(i, j, k) >= 1.0; }
    }

    inline double _randomFloat(double min, double max) {
        return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
    }

    bool _isSimulationInitialized = false;
    bool _isSimulationRunning = false;
    bool _isFluidInSimulation = false;

    int MESH = 0;
    int IMPLICIT = 1;
    int _fluidInitializationType = 2;
    std::string _fluidMeshFilename;
    glm::vec3 _fluidMeshOffset;
    double _fluidMeshScale = 1.0;
    

    int _currentFrame = 0;
    int _currentTimeStep = 0;
    bool _isCurrentFrameFinished = true;

    double _dx = 0.0;
    double _density = 20.0;
    int _i_voxels = 0;
    int _j_voxels = 0;
    int _k_voxels = 0;

    double _CFLConditionNumber = 5.0;
    double _minTimeStep = 1.0 / 1200.0;
    double _maxTimeStep = 1.0 / 15.0;
    double _maxAdvectionDistanceFactor = 2.5; // max number of cells an advection
                                              // integration can travel
    double _pressureSolveTolerance = 10e-6;
    int _maxPressureSolveIterations = 300;
    int _numAdvanceMarkerParticleThreads = 8;

    glm::vec3 _bodyForce;

    MACVelocityField _MACVelocity;
    Array3d<int> _materialGrid;
    Array3d<double> _pressureGrid;
    Array3d<int> _layerGrid;

    MatrixCoefficients _matrixA;
    VectorCoefficients _preconditioner;
    Array3d<int> _GridIndexToEigenVectorXdIndex;

    ImplicitSurfaceScalarField _implicitFluidScalarField;
    LevelSetField _levelsetField;

    std::vector<MarkerParticle> _markerParticles;
    std::vector<GridIndex> _fluidCellIndices;

    LogFile _logfile;
    double _simulationTime = 0;
    double _realTime = 0;

    TriangleMesh _surfaceMesh;
    LevelSet _levelset;

    double _surfaceReconstructionSmoothingValue = 0.85;
    int _surfaceReconstructionSmoothingIterations = 3;
    double _markerParticleRadius;
    double _markerParticleScale = 3.0;
    
    std::vector<FluidSource*> _fluidSources;
    std::vector<SphericalFluidSource*> _sphericalFluidSources;
    std::vector<CuboidFluidSource*> _cuboidFluidSources;

    double _diffuseSurfaceNarrowBandSize = 0.25; // size in # of cells
    double _minWavecrestCurvature = 0.5;
    double _maxWavecrestCurvature = 2.0;
    double _minParticleEnergy = 0.0;
    double _maxParticleEnergy = 15.0;
    double _minTurbulence = 20.0;
    double _maxTurbulence = 50.0;
};