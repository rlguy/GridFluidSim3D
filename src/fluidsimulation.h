/*
Copyright (c) 2016 Ryan L. Guy

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
#ifndef FLUIDSIMULATION_H
#define FLUIDSIMULATION_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <assert.h>

#include "stopwatch.h"
#include "macvelocityfield.h"
#include "array3d.h"
#include "grid3d.h"
#include "implicitsurfacescalarfield.h"
#include "clscalarfield.h"
#include "polygonizer3d.h"
#include "trianglemesh.h"
#include "logfile.h"
#include "collision.h"
#include "aabb.h"
#include "levelset.h"
#include "fluidsimulationsavestate.h"
#include "fluidsource.h"
#include "sphericalfluidsource.h"
#include "cuboidfluidsource.h"
#include "turbulencefield.h"
#include "diffuseparticlesimulation.h"
#include "fluidbrickgrid.h"
#include "spatialpointgrid.h"
#include "isotropicparticlemesher.h"
#include "anisotropicparticlemesher.h"
#include "gridindexkeymap.h"
#include "pressuresolver.h"
#include "particleadvector.h"
#include "fluidmaterialgrid.h"
#include "gridindexvector.h"
#include "fragmentedvector.h"
#include "vmath.h"

#include "markerparticle.h"
#include "diffuseparticle.h"

class FluidSimulation
{
public:
    FluidSimulation();
    FluidSimulation(int isize, int jsize, int ksize, double dx);
    FluidSimulation(FluidSimulationSaveState &state);
    ~FluidSimulation();

    void initialize();
    void update(double dt);
    void saveState(std::string filename);
    int getCurrentFrame();
    bool isCurrentFrameFinished();

    double getCellSize();
    void getGridDimensions(int *i, int *j, int *k);
    void getSimulationDimensions(double *w, double *h, double *d);
    double getSimulationWidth();
    double getSimulationHeight();
    double getSimulationDepth();
    double getDensity();
    void setDensity(double p);
    Material getMaterial(int i, int j, int k);
    void setMarkerParticleScale(double s);
    void setSurfaceSubdivisionLevel(unsigned int n);
    void setNumSurfaceReconstructionPolygonizerSlices(int n);
    void setMinimumPolyhedronTriangleCount(unsigned int n);

    void enableSurfaceMeshOutput();
    void disableSurfaceMeshOutput();
    void enableIsotropicSurfaceReconstruction();
    void disableIsotropicSurfaceReconstruction();
    void enableAnisotropicSurfaceReconstruction();
    void disableAnisotropicSurfaceReconstruction();
    void enableDiffuseMaterialOutput();
    void enableBubbleDiffuseMaterial();
    void enableSprayDiffuseMaterial();
    void enableFoamDiffuseMaterial();
    void disableBubbleDiffuseMaterial();
    void disableSprayDiffuseMaterial();
    void disableFoamDiffuseMaterial();
    void outputDiffuseMaterialAsSeparateFiles();
    void outputDiffuseMaterialAsSingleFile();
    void disableDiffuseMaterialOutput();
    void enableBrickOutput();
    void enableBrickOutput(double width, double height, double depth);
    void disableBrickOutput();
    void enableAutosave();
    void disableAutosave();

    void addBodyForce(double fx, double fy, double fz);
    void addBodyForce(vmath::vec3 f);
    void addBodyForce(vmath::vec3 (*fieldFunction)(vmath::vec3));
    void resetBodyForces();

    void addImplicitFluidPoint(double x, double y, double z, double r);
    void addImplicitFluidPoint(vmath::vec3 p, double radius);
    void addFluidCuboid(double x, double y, double z, double w, double h, double d);
    void addFluidCuboid(vmath::vec3 p, double width, double height, double depth);
    void addFluidCuboid(vmath::vec3 p1, vmath::vec3 p2);

    SphericalFluidSource *addSphericalFluidSource(vmath::vec3 pos, double r);
    SphericalFluidSource *addSphericalFluidSource(vmath::vec3 pos, double r, 
                                                  vmath::vec3 velocity);
    CuboidFluidSource *addCuboidFluidSource(AABB bbox);
    CuboidFluidSource *addCuboidFluidSource(AABB bbox, vmath::vec3 velocity);
    void removeFluidSource(FluidSource *source);
    void removeFluidSources();

    void addSolidCell(int i, int j, int k);
    void addSolidCell(GridIndex g);
    void addSolidCells(std::vector<GridIndex> indices);
    void removeSolidCell(int i, int j, int k);
    void removeSolidCells(std::vector<vmath::vec3> indices);
    std::vector<vmath::vec3> getSolidCells();
    std::vector<vmath::vec3> getSolidCellPositions();
    void addFluidCell(int i, int j, int k);
    void addFluidCell(GridIndex g);
    void addFluidCells(GridIndexVector indices);

    unsigned int getNumMarkerParticles();
    std::vector<vmath::vec3> getMarkerParticlePositions();
    std::vector<vmath::vec3> getMarkerParticlePositions(int startidx, int endidx);
    std::vector<vmath::vec3> getMarkerParticleVelocities();
    std::vector<vmath::vec3> getMarkerParticleVelocities(int startidx, int endidx);
    unsigned int getNumDiffuseParticles();
    void getDiffuseParticles(std::vector<DiffuseParticle> &dps);
    std::vector<vmath::vec3> getDiffuseParticlePositions();
    std::vector<vmath::vec3> getDiffuseParticlePositions(int startidx, int endidx);
    std::vector<vmath::vec3> getDiffuseParticleVelocities();
    std::vector<vmath::vec3> getDiffuseParticleVelocities(int startidx, int endidx);
    std::vector<float> getDiffuseParticleLifetimes();
    std::vector<float> getDiffuseParticleLifetimes(int startidx, int endidx);
    std::vector<char> getDiffuseParticleTypes();
    std::vector<char> getDiffuseParticleTypes(int startidx, int endidx);
    Array3d<float> getDensityGrid();
    MACVelocityField* getVelocityField();
    LevelSet* getLevelSet();
    TriangleMesh* getFluidSurfaceTriangles();

private:

    struct DiffuseParticleEmitter {
        vmath::vec3 position;
        vmath::vec3 velocity;
        double energyPotential;
        double wavecrestPotential;
        double turbulencePotential;

        DiffuseParticleEmitter() : energyPotential(0.0),
                                   wavecrestPotential(0.0),
                                   turbulencePotential(0.0) {}

        DiffuseParticleEmitter(vmath::vec3 p, vmath::vec3 v, 
                               double e, double wc, double t) : 
                                   position(p),
                                   velocity(v),
                                   energyPotential(e),
                                   wavecrestPotential(wc),
                                   turbulencePotential(t) {}
    };    

    struct FluidPoint {
        vmath::vec3 position;
        double radius = 0.0;

        FluidPoint() {}
        FluidPoint(vmath::vec3 p, double r) : position(p),
                                              radius(r) {}
    };

    struct FluidCuboid {
        AABB bbox;

        FluidCuboid() {}
        FluidCuboid(vmath::vec3 p, double w, double h, double d) : 
                        bbox(p, w, h, d) {}
    };

    // Type constants
    int DP_BUBBLE = 0;
    int DP_FOAM = 1;
    int DP_SPRAY = 2;
    int DP_NOTSET = -1;

    // Initialization before running simulation
    void _initializeSimulation();
    void _initializeSolidCells();
    void _initializeFluidMaterial();
    void _calculateInitialFluidSurfaceScalarField(ImplicitSurfaceScalarField &field);
    void _getInitialFluidCellsFromScalarField(ImplicitSurfaceScalarField &field,
                                              GridIndexVector &fluidCells);
    void _getPartiallyFilledFluidCellParticles(GridIndexVector &partialFluidCells,
                                               ImplicitSurfaceScalarField &field,
                                               std::vector<vmath::vec3> &partialParticles);
    void _initializeMarkerParticles(GridIndexVector &fullFluidCells,
                                    std::vector<vmath::vec3> &partialParticles);
    void _initializeFluidCellIndices();
    void _addMarkerParticlesToCell(GridIndex g);
    void _addMarkerParticlesToCell(GridIndex g, vmath::vec3 velocity);
    void _addMarkerParticle(vmath::vec3 p);
    void _addMarkerParticle(vmath::vec3 p, vmath::vec3 velocity);
    void _addMarkerParticles(std::vector<vmath::vec3> particles);
    void _initializeSimulationFromSaveState(FluidSimulationSaveState &state);
    void _initializeMarkerParticlesFromSaveState(FluidSimulationSaveState &state);
    void _initializeDiffuseParticlesFromSaveState(FluidSimulationSaveState &state);
    void _initializeFluidMaterialParticlesFromSaveState();
    void _initializeSolidCellsFromSaveState(FluidSimulationSaveState &state);
    void _initializeCLObjects();

    // Simulation step
    double _calculateNextTimeStep();
    double _getMaximumMarkerParticleSpeed();
    void _autosave();
    void _stepFluid(double dt);

    // Find fluid cells. Fluid cells must contain at
    // least 1 marker particle
    int _getUniqueFluidSourceID();
    void _updateFluidCells();
    void _updateAddedFluidCellQueue();
    void _updateFluidSources();
    void _updateInflowFluidSource(FluidSource *source);
    void _addNewFluidCells(GridIndexVector &cells, vmath::vec3 velocity);
    void _addNewFluidParticles(std::vector<vmath::vec3> &particles, vmath::vec3 velocity);
    void _getNewFluidParticles(FluidSource *source, std::vector<vmath::vec3> &particles);
    void _removeMarkerParticlesFromCells(Array3d<bool> &isRemovalCell);
    void _removeDiffuseParticlesFromCells(Array3d<bool> &isRemovalCell);

    // Convert marker particles to fluid surface
    void _reconstructFluidSurface();
    TriangleMesh _polygonizeSurface();

    // Update level set surface
    void _updateLevelSetSignedDistance();

    // Reconstruct output fluid surface
    void _reconstructOutputFluidSurface(double dt);
    std::string _numberToString(int number);
    void _writeSurfaceMeshToFile(TriangleMesh &isomesh,
                                 TriangleMesh &anisomesh);
    void _writeDiffuseMaterialToFile(std::string bubblefile,
                                     std::string foamfile,
                                     std::string sprayfile);
    void _writeDiffuseMaterialToFile(std::string diffusefile);
    void _writeBrickColorListToFile(TriangleMesh &mesh, std::string filename);
    void _writeBrickTextureToFile(TriangleMesh &mesh, std::string filename);
    void _writeBrickMaterialToFile(std::string brickfile, 
                                   std::string colorfile, 
                                   std::string texturefile);
    void _smoothSurfaceMesh(TriangleMesh &mesh);
    void _getSmoothVertices(TriangleMesh &mesh, std::vector<int> &smoothVertices);
    bool _isVertexNearSolid(vmath::vec3 v, double eps);
    TriangleMesh _polygonizeIsotropicOutputSurface();
    TriangleMesh _polygonizeAnisotropicOutputSurface();
    void _updateBrickGrid(double dt);

    // Advect fluid velocities
    void _advectVelocityField();
    void _advectVelocityFieldU();
    void _advectVelocityFieldV();
    void _advectVelocityFieldW();
    void _computeVelocityScalarField(Array3d<float> &field, 
                                     Array3d<bool> &isValueSet, 
                                     int dir);
    void _applyFluidSourceToVelocityField(FluidSource *source,
                                          int dir,
                                          Array3d<bool> &isValueSet,
                                          Array3d<float> &field);

    // Add gravity to fluid velocities
    void _applyBodyForcesToVelocityField(double dt);
    vmath::vec3 _getConstantBodyForce();
    void _applyConstantBodyForces(double dt);
    void _applyVariableBodyForces(double dt);
    void _applyVariableBodyForce(vmath::vec3 (*fieldFunction)(vmath::vec3),
                                 double dt);

    // Extrapolate fluid velocities into surrounding air and solids so
    // that velocities can be computed when marker particles move to cells
    // outside of current fluid region
    void _extrapolateFluidVelocities(MACVelocityField &MACGrid);

    // Calculate pressure values to satisfy incompressibility condition
    void _updatePressureGrid(Array3d<float> &pressureGrid, double dt);

    // Alter fluid velocities according to calculated pressures
    // to create a divercence free velocity field
    void _applyPressureToVelocityField(Array3d<float> &pressureGrid, double dt);
    void _applyPressureToFaceU(int i, int j, int k, Array3d<float> &pressureGrid,
                                                    MACVelocityField &tempMACVelocity, double dt);
    void _applyPressureToFaceV(int i, int j, int k, Array3d<float> &pressureGrid,
                                                    MACVelocityField &tempMACVelocity, double dt);
    void _applyPressureToFaceW(int i, int j, int k, Array3d<float> &pressureGrid,
                                                    MACVelocityField &tempMACVelocity, double dt);
    void _commitTemporaryVelocityFieldValues(MACVelocityField &tempMACVelocity);

    // Update diffuse material (spray, foam, bubbles)
    void _updateDiffuseMaterial(double dt);

    // Transfer grid velocity to marker particles
    void _updateMarkerParticleVelocities();
    void _updateRangeOfMarkerParticleVelocities(int startIdx, int endIdx);

    // Move marker particles through the velocity field
    void _advanceMarkerParticles(double dt);
    void _advanceRangeOfMarkerParticles(int startIdx, int endIdx, double dt);
    vmath::vec3 _resolveParticleSolidCellCollision(vmath::vec3 p0, vmath::vec3 p1);
    void _removeMarkerParticles();
    void _shuffleMarkerParticleOrder();

    template<class T>
    void _removeItemsFromVector(std::vector<T> &items, std::vector<bool> &isRemoved) {
        assert(items.size() == isRemoved.size());

        int currentidx = 0;
        for (unsigned int i = 0; i < items.size(); i++) {
            if (!isRemoved[i]) {
                items[currentidx] = items[i];
                currentidx++;
            }
        }

        items.erase(items.begin() + currentidx, items.end());
        items.shrink_to_fit();
    }

    template<class T>
    void _removeItemsFromVector(FragmentedVector<T> &items, std::vector<bool> &isRemoved) {
        assert(items.size() == isRemoved.size());

        int currentidx = 0;
        for (unsigned int i = 0; i < items.size(); i++) {
            if (!isRemoved[i]) {
                items[currentidx] = items[i];
                currentidx++;
            }
        }

        for (int i = 0; i < items.size() - currentidx; i++) {
            items.pop_back();
        }
        items.shrink_to_fit();
    }

    template<class T>
    void _removeItemsFromVector(FragmentedVector<T> *items, std::vector<bool> &isRemoved) {
        assert(items->size() == isRemoved.size());

        int currentidx = 0;
        for (unsigned int i = 0; i < items->size(); i++) {
            if (!isRemoved[i]) {
                (*items)[currentidx] = (*items)[i];
                currentidx++;
            }
        }

        for (int i = 0; i < items->size() - currentidx; i++) {
            items->pop_back();
        }
        items->shrink_to_fit();
    }

    inline double _randomDouble(double min, double max) {
        return min + (double)rand() / ((double)RAND_MAX / (max - min));
    }

    bool _isSimulationInitialized = false;
    bool _isSimulationRunning = false;
    bool _isFluidInSimulation = false;
    int _currentFrame = 0;
    int _currentTimeStep = 0;
    double _currentDeltaTime = 0.0;
    double _frameTimeStep = 0.0;
    bool _isCurrentFrameFinished = true;
    bool _isLastTimeStepForFrame = false;
    double _simulationTime = 0;
    double _realTime = 0;
    int _loadStateReadChunkSize = 50000;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    double _CFLConditionNumber = 5.0;
    double _minTimeStep = 1.0 / 1200.0;
    double _maxTimeStep = 1.0 / 15.0;
    int _maxParticlesPerAdvectionComputation = 5e6;

    double _density = 20.0;
    int _maxParticlesPerAdvection = 10e6;
    int _maxParticlesPerVelocityUpdate = 10e6;
    MACVelocityField _savedVelocityField;

    double _surfaceReconstructionSmoothingValue = 0.5;
    int _surfaceReconstructionSmoothingIterations = 2;
    int _minimumSurfacePolyhedronTriangleCount = 0;
    double _markerParticleRadius;
    double _markerParticleScale = 3.0;

    int _outputFluidSurfaceSubdivisionLevel = 1;
    int _numSurfaceReconstructionPolygonizerSlices = 1;
    double _outputFluidSurfaceCellNarrowBandSize = 0.5;
    double _outputFluidSurfaceParticleNarrowBandSize = 1.0;
    int _outputFluidSurfacePolygonizerChunkSize = 128*128*64;
    double _outputFluidSurfacePolygonizerChunkPad = 3.0; // in # of cells

    double _ratioPICFLIP = 0.05f;
    int _maxMarkerParticlesPerCell = 100;

    bool _isSurfaceMeshOutputEnabled = true;
    bool _isIsotropicSurfaceMeshReconstructionEnabled = true;
    bool _isAnisotropicSurfaceMeshReconstructionEnabled = false;
    bool _isDiffuseMaterialOutputEnabled = false;
    bool _isBubbleDiffuseMaterialEnabled = false;
    bool _isSprayDiffuseMaterialEnabled = false;
    bool _isFoamDiffuseMaterialEnabled = false;
    bool _isDiffuseMaterialFilesSeparated = false;
    bool _isBrickOutputEnabled = false;
    bool _isAutosaveEnabled = true;
    double _brickWidth = 1.0;
    double _brickHeight = 1.0;
    double _brickDepth = 1.0;
    int _currentBrickMeshFrame = 0;

    std::vector<vmath::vec3> _constantBodyForces;

    typedef vmath::vec3 (*FieldFunction)(vmath::vec3);
    std::vector<FieldFunction> _variableBodyForces;

    MACVelocityField _MACVelocity;
    FluidMaterialGrid _materialGrid;
    FragmentedVector<MarkerParticle> _markerParticles;
    GridIndexVector _fluidCellIndices;
    GridIndexVector _addedFluidCellQueue;
    LogFile _logfile;
    TriangleMesh _surfaceMesh;
    LevelSet _levelset;
    DiffuseParticleSimulation _diffuseMaterial;
    
    std::vector<FluidPoint> _fluidPoints;
    std::vector<FluidCuboid> _fluidCuboids;
    std::vector<FluidSource*> _fluidSources;
    std::vector<SphericalFluidSource*> _sphericalFluidSources;
    std::vector<CuboidFluidSource*> _cuboidFluidSources;
    int _uniqueFluidSourceID = 0;

    Array3d<Brick> _brickGrid;
    FluidBrickGrid _fluidBrickGrid;

    ParticleAdvector _particleAdvector;
    CLScalarField _scalarFieldAccelerator;

};

#endif