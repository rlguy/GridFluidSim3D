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

    /*
        Constructs a FluidSimulation object with grid dimensions
        of width isize, height jsize, depth ksize, with each grid cell
        having width dx.
    */
    FluidSimulation(int isize, int jsize, int ksize, double dx);

    /*
        Constructs a FluidSimulation object from a saved state.

        Example usage:

            std::string filename = "savestates/autosave.state";
            FluidSimulationSaveState state;
            assert(state.loadState(filename));
            FluidSimulation fluidsim(state);
    */
    FluidSimulation(FluidSimulationSaveState &state);

    ~FluidSimulation();

    /*
        Initializes the fluid simulation.

        Must be called before running update() method.

        Calls to addImplicitFluidPoint() and addFluidCuboid() must
        be made before this method is run.
    */
    void initialize();

    /*
        Advance the fluid simulation for a single frame time of dt seconds.
    */
    void update(double dt);

    /*
        Save the current state of the simulation as a file.
    */
    void saveState(std::string filename);

    /*
        Returns current frame of the simulation. Frame numbering starts
        at zero.
    */
    int getCurrentFrame();

    /*
        Returns false only when the update() method is being executed. May be
        used if executing the update() method in seperate thread.
    */
    bool isCurrentFrameFinished();

    /* 
        returns the width of a simulation grid cell
    */
    double getCellSize();

    /*
        Retrieves the simulation grid dimensions.
            i   ->   width
            j   ->   height
            k   ->   depth
    */
    void getGridDimensions(int *i, int *j, int *k);

    /*  
        Retrieves the physical simulation dimensions. Values are equivalent
        to multiplying the grid dimension by the grid cell size.
    */
    void getSimulationDimensions(double *width, double *height, double *depth);
    double getSimulationWidth();
    double getSimulationHeight();
    double getSimulationDepth();

    /*
        Density of the fluid. 
        Must be greater than zero. 
        Fluid density currently has no effect on the simulation.
    */
    double getDensity();
    void setDensity(double p);

    /*
        Returns the material type stored in a grid cell.
        Material type can be Material::fluid, Material::solid, or Material::air
    */
    Material getMaterial(int i, int j, int k);
    Material getMaterial(GridIndex g);

    /*
        Marker particle scale determines how large a particle is when
        converting a set of particles to a triangle mesh. 

        A marker particle with a scale of 1.0 will have the radius of a 
        sphere that has a volume 1/8th of the volume of a grid cell.
    */
    void setMarkerParticleScale(double s);
    double getMarkerParticleScale();

    /*
        The surface subdivision level determines how many times the
        simulation grid is divided when converting marker particles
        to a triangle mesh

        For example, a simulation with dimensions 256 x 128 x 80 and
        a subdivision level of 2 will polygonize the surface on a grid
        with dimensions 512 x 256 x 160. With a subdivision of level 3,
        the polygonization grid will have dimensions 768 x 384 x 240.

        A higher subdivision level will produce a higher quality surface at
        the cost of longer simulation times and greater memory usage.
    */
    void setSurfaceSubdivisionLevel(int n);

    /*
        How many slices the polygonizer will section the surface reconstruction
        grid into when computing the triangle mesh. The polygonizer will compute
        the triangle mesh one slice at a time before combining them into a
        single triangle mesh.

        A higher subdivision level may require a very large amount of memory to
        store the polygonization grid data. Setting the number of slices will 
        reduce the memory required at the cost of speed.
    */
    void setNumSurfaceReconstructionPolygonizerSlices(int n);

    /*
        Will ensure that the output triangle mesh only contains polyhedrons
        that contain a minimum number of triangles. Removing polyhedrons with
        a low triangle count will reduce the triangle mesh size when saved to 
        disk.
    */
    void setMinimumPolyhedronTriangleCount(int n);

    /*
        Enable/disable the simulation from saving polygonized triangle meshes 
        to disk. 

        Enabled by default.
    */
    void enableSurfaceMeshOutput();
    void disableSurfaceMeshOutput();
    bool isSurfaceMeshOutputEnabled();

    /*
        Enable/disable the simulation from saving isotropic reconstructed triangle 
        meshes to disk.

        Isotropic surface reconstruction creates a triangle mesh from a set of
        spheres each with a uniform radius.

        Enabled by default.
    */
    void enableIsotropicSurfaceReconstruction();
    void disableIsotropicSurfaceReconstruction();
    bool isIsotropicSurfaceReconstuctionEnabled();

    /*
        Enable/disable the simulation from saving anisotropic reconstructed triangle 
        meshes to disk.

        Anisotropic surface reconstruction creates a triangle mesh from a set of
        ellipsoids (3d ovals). This method preserves sharp features of the fluid
        by converting a set of particles to ellipsoids that better match a smooth
        fluid surface. This method of surface reconstruction is much slower than
        the isotropic method.

        Disabled by default.
    */
    void enableAnisotropicSurfaceReconstruction();
    void disableAnisotropicSurfaceReconstruction();
    bool isAnisotropicSurfaceReconstuctionEnabled();

    /*
        Enable/disable the simulation from simulating diffuse 
        material (spray/bubble/foam particles), and saving diffuse mesh data to disk.

        Diffuse material mesh data consists of triangle meshes containing only vertices.

        Disabled by default.
    */
    void enableDiffuseMaterialOutput();
    void disableDiffuseMaterialOutput();
    bool isDiffuseMaterialOutputEnabled();

    /*
        Enable/disable the simulation from saving diffuse bubble mesh data to disk.

        Enabled by default if diffuse material output is enabled
    */
    void enableBubbleDiffuseMaterial();
    void disableBubbleDiffuseMaterial();
    bool isBubbleDiffuseMaterialEnabled();

    /*
        Enable/disable the simulation from saving diffuse spray mesh data to disk.

        Enabled by default if diffuse material output is enabled
    */
    void enableSprayDiffuseMaterial();
    void disableSprayDiffuseMaterial();
    bool isSprayDiffuseMaterialEnabled();

    /*
        Enable/disable the simulation from saving diffuse foam mesh data to disk.

        Enabled by default if diffuse material output is enabled
    */
    void enableFoamDiffuseMaterial();
    void disableFoamDiffuseMaterial();
    bool isFoamDiffuseMaterialEnabled();

    /*
        Save diffuse material to disk as a single file per frame.

        Enabled by default.
    */
    void outputDiffuseMaterialAsSingleFile();

    /*
        Save diffuse material to disk as a multiple files per frame. Files
        will be separated by diffuse particle type (spray, bubble, foam).

        Disabled by default.
    */
    void outputDiffuseMaterialAsSeparateFiles();

    /*
        Enable/disable the simulation from simulating the fluid as a set of
        'LEGO' bricks and saving brick data to disk.

        When enabling brick output, the width, heigh, and depth dimension of the
        brick must be specified.

        Disabled by default.
    */
    void enableBrickOutput(double width, double height, double depth);
    void enableBrickOutput(AABB brickbbox);
    void disableBrickOutput();
    bool isBrickOutputEnabled();

    /*
        Returns whether the the FluidBrickGrid is initialized. Will be
        initialized if enableBrickOutput() has been called. This function
        can be used to check whether a FluidSimulation loaded from a save
        state has also loaded and initialized the FluidBrickGrid.
    */
    bool isFluidBrickGridInitialized();

    /*
        Returns dimensions of the brick used in the brick output feature.
        Dimensions are only valid if isFluidBrickGridInitialized() returns 
        true;
    */
    AABB getBrickAABB();

    /*
        Enable/disable autosaving the state of the simulation at the start
        of each frame.

        Enabled by default.
    */
    void enableAutosave();
    void disableAutosave();
    bool isAutosaveEnabled();


    /*
        Add a constant force such as gravity to the simulation.
    */
    void addBodyForce(double fx, double fy, double fz);
    void addBodyForce(vmath::vec3 f);

    /*
        Add a variable body force field function to the simulation. 
        The force field function takes a 3d vector position as a parameter
        and returns a 3d vector force.

        Example field function:

            vmath::vec3 forceField(vmath::vec3 p) {
                vmath::vec3 forceVector(0.0, -9.8, 0.0);

                if (p.x < 4.0) {
                    forceVector.y = -forceVector.y;
                }
                
                return forceVector
            }
    */
    void addBodyForce(vmath::vec3 (*fieldFunction)(vmath::vec3));

    /*
        Remove all added body forces.
    */
    void resetBodyForces();

    /*
        Add an implicit point of fluid to the simulation. 

        An implicit fluid point is represented as a field of scalar values on 
        the simulation grid. The strength of the field values are 1 at the point center 
        and falls off at a cubic rate towards 0 as distance from the center 
        increases. When the simulation is initialized, fluid particles will be created
        in regions where the scalar field values are greater than 0.5.

        Calls to these methods must be executed before calling initialize().
    */
    void addImplicitFluidPoint(double x, double y, double z, double r);
    void addImplicitFluidPoint(vmath::vec3 p, double radius);
    
    /*
        Add a cuboid of fluid to the simulation.

        Calls to these methods must be executed before calling initialize().
    */
    void addFluidCuboid(double x, double y, double z, double w, double h, double d);
    void addFluidCuboid(vmath::vec3 p1, vmath::vec3 p2);
    void addFluidCuboid(AABB bbox);
    void addFluidCuboid(vmath::vec3 p, double width, double height, double depth);

    /*
        Add a spherical shaped fluid source with position pos, radius r, and an optional 
        emission velocity to the simulation and return a pointer to the source object.

        A fluid source can be either of type inflow (emit fluid) or outflow (remove fluid) 
        and type can be set using source->setAsInFlow() and source->setAsOutFlow() 
        respectively.

        Fluid sources are of type inflow by default.
    */
    SphericalFluidSource *addSphericalFluidSource(vmath::vec3 pos, double r);
    SphericalFluidSource *addSphericalFluidSource(vmath::vec3 pos, double r, 
                                                  vmath::vec3 velocity);

    /*
        Add a cuboid shaped fluid source matching the position and dimensions of bbox 
        to the simulation and return a pointer to the source object.

        An axis aligned bounding box object can be initialized in the following manner:

            AABB bbox(vmath::vec3(x, y, z), width, height, depth);

        where x, y, z are the position coordinates of the minimal point of the AABB
        and width, height, depth are the dimensions according to the x,y,z directions.

        A fluid source can be either of type inflow (emit fluid) or outflow (remove fluid) 
        and type can be set using source->setAsInFlow() and source->setAsOutFlow() 
        respectively.

        Fluid sources are of type inflow by default.
    */
    CuboidFluidSource *addCuboidFluidSource(AABB bbox);
    CuboidFluidSource *addCuboidFluidSource(AABB bbox, vmath::vec3 velocity);

    /*
        Remove a fluid source from the simulation. The pointer will become invalid
        after this method executes.
    */
    void removeFluidSource(FluidSource *source);

    /*
        Remove all fluid sources from the simulation.
    */
    void removeFluidSources();

    /*
        Add solid cells to the simulation grid. If a solid cell is added
        to a region containing fluid particles, those fluid particles will
        be removed from the simulation.
    */
    void addSolidCell(int i, int j, int k);
    void addSolidCell(GridIndex g);
    void addSolidCells(std::vector<GridIndex> &indices);

    /*
        Remove solid cells from the simulation grid. When a solid cell is
        removed, the material will be replaced by air.

        The bordering cells of the simulation grid are permanently set as
        solid cells and will not be removed.
    */
    void removeSolidCell(int i, int j, int k);
    void removeSolidCells(std::vector<GridIndex> &indices);

    /*
        Returns a vector containing the indices of all solid cells.
    */
    std::vector<GridIndex> getSolidCells();

    /*
        Returns a vector containing the position of the minimal grid cell
        corner of all solid cells.
    */
    std::vector<vmath::vec3> getSolidCellPositions();

    /*
        Add fluid cells to the simulation grid. Fluid cells will only be
        added if the current cell material is of type air.
    */
    void addFluidCell(int i, int j, int k);
    void addFluidCell(GridIndex g);
    void addFluidCells(GridIndexVector &indices);

    /*
        Returns the number of marker particles in the simulation. Marker particles
        track where the fluid is and carry velocity data.
    */
    unsigned int getNumMarkerParticles();

    /*
        Returns a vector of all marker particles in the simulation.
    */
    void getMarkerParticles(std::vector<MarkerParticle> &mps);

    /*
        Returns a vector of marker particle positions. If range indices
        are specified, the vector will contain positions ranging from 
        start index startidx and ending at endidx inclusively.
    */
    std::vector<vmath::vec3> getMarkerParticlePositions();
    std::vector<vmath::vec3> getMarkerParticlePositions(int startidx, int endidx);

    /*
        Returns a vector of marker particle velocities. If range indices
        are specified, the vector will contain velocities ranging from 
        start index startidx and ending at endidx inclusively.
    */
    std::vector<vmath::vec3> getMarkerParticleVelocities();
    std::vector<vmath::vec3> getMarkerParticleVelocities(int startidx, int endidx);

    /*
        Returns the number of diffuse particles in the simulation. Diffuse particles
        have a position, velocity, lifetime value, and can be of type bubble, spray,
        or foam.
    */
    unsigned int getNumDiffuseParticles();

    /*
        Returns a vector of all diffuse particles in the simulation.
    */
    void getDiffuseParticles(std::vector<DiffuseParticle> &dps);

    /*
        Returns a vector of diffuse particle positions. If range indices
        are specified, the vector will contain positions ranging from 
        start index startidx and ending at endidx inclusively.
    */
    std::vector<vmath::vec3> getDiffuseParticlePositions();
    std::vector<vmath::vec3> getDiffuseParticlePositions(int startidx, int endidx);

    /*
        Returns a vector of diffuse particle velocities. If range indices
        are specified, the vector will contain velocities ranging from 
        start index startidx and ending at endidx inclusively.
    */
    std::vector<vmath::vec3> getDiffuseParticleVelocities();
    std::vector<vmath::vec3> getDiffuseParticleVelocities(int startidx, int endidx);

    /*
        Returns a vector of diffuse particle lifetimes. If range indices
        are specified, the vector will contain remaining lifetimes (in seconds) 
        ranging from start index startidx and ending at endidx inclusively.
    */
    std::vector<float> getDiffuseParticleLifetimes();
    std::vector<float> getDiffuseParticleLifetimes(int startidx, int endidx);

    /*
        Returns a vector of diffuse particle types. If range indices
        are specified, the vector will contain types ranging from 
        start index startidx and ending at endidx inclusively.

        Char value and corresponding diffuse particle type:

            0x00    Bubble
            0x01    Foam
            0x02    Spray
    */
    std::vector<char> getDiffuseParticleTypes();
    std::vector<char> getDiffuseParticleTypes(int startidx, int endidx);

    /*
        Returns a pointer to the MACVelocityField data structure.
        The MAC velocity field is a staggered velocity field that
        stores velocity components at the location of face centers.
    */
    MACVelocityField* getVelocityField();

    /*
        Returns a pointer to the LevelSet data structure. The levelset
        is used for querying distance to the fluid surface at a point.
    */
    LevelSet* getLevelSet();

    /*
        Returns a pointer to the FluidBrickGrid data structure. The
        FluidBrickGrid is used for converting the fluid surface into
        a set of colored cuboids for use in a rendering effect.
    */
    FluidBrickGrid* getFluidBrickGrid();

private:   

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
    void _initializeFluidBrickGridFromSaveState(FluidSimulationSaveState &state);
    void _initializeCLObjects();

    // Simulation step
    double _calculateNextTimeStep();
    double _getMaximumMarkerParticleSpeed();
    void _autosave();
    void _printError(std::string msg);
    void _stepFluid(double dt);

    // Find fluid cells. Fluid cells must contain at
    // least 1 marker particle
    int _getUniqueFluidSourceID();
    void _updateFluidCells();
    void _removeParticlesInSolidCells();
    void _removeMarkerParticlesInSolidCells();
    void _removeDiffuseParticlesInSolidCells();
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
    void _removeItemsFromVector(std::vector<T> *items, std::vector<bool> &isRemoved) {
        _removeItemsFromVector(*items, isRemoved);
    }

    template<class T>
    void _removeItemsFromVector(FragmentedVector<T> *items, std::vector<bool> &isRemoved) {
        _removeItemsFromVector(*items, isRemoved);
    }

    inline double _randomDouble(double min, double max) {
        return min + (double)rand() / ((double)RAND_MAX / (max - min));
    }

    bool _isSimulationInitialized = false;
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
    int _currentBrickMeshFrame = 0;
    int _brickMeshFrameOffset = -3;

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