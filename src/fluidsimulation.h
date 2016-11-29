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

#include "stopwatch.h"
#include "macvelocityfield.h"
#include "array3d.h"
#include "grid3d.h"
#include "scalarfield.h"
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
#include "fluidsimassert.h"
#include "config.h"

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
            FLUIDSIM_ASSERT(state.loadState(filename));
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
    bool isInitialized();

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
    int getGridWidth();
    int getGridHeight();
    int getGridDepth();

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
    double getMarkerParticleScale();
    void setMarkerParticleScale(double s);

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
    int getSurfaceSubdivisionLevel();
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
    int getNumPolygonizerSlices();
    void setNumPolygonizerSlices(int n);

    /*
        Will ensure that the output triangle mesh only contains polyhedrons
        that contain a minimum number of triangles. Removing polyhedrons with
        a low triangle count will reduce the triangle mesh size when saved to 
        disk.
    */
    int getMinPolyhedronTriangleCount();
    void setMinPolyhedronTriangleCount(int n);

    /*
        Offset will be added to the position of the meshes output by the 
        simulator.
    */
    vmath::vec3 getDomainOffset();
    void setDomainOffset(double x, double y, double z);
    void setDomainOffset(vmath::vec3 offset);

    /*
        Specify file format that meshes will be written as. The simulator will
        write .PLY meshes by default.
    */
    void setMeshOutputFormatAsPLY();
    void setMeshOutputFormatAsBOBJ();

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
    bool isIsotropicSurfaceReconstructionEnabled();

    /*
        Enable/disable the simulation from saving preview triangle 
        meshes to disk.

        A preview mesh is reconstructed at a specified resolution seperate from
        the simulation resolution.

        Enabled by default.
    */
    void enablePreviewMeshOutput(double dx);
    void disablePreviewMeshOutput();
    bool isPreviewMeshOutputEnabled();

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
    bool isAnisotropicSurfaceReconstructionEnabled();

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
        The number of diffuse particles simulated in the diffuse particle
        simulation will be limited by this number.
    */
    int getMaxNumDiffuseParticles();
    void setMaxNumDiffuseParticles(int n);

    /*
        The maximum lifetime of a diffuse particle is spawned for in 
        seconds. Set this value to control how quickly/slowly diffuse
        particles fade from the simulation.
    */
    double getMaxDiffuseParticleLifetime();
    void setMaxDiffuseParticleLifetime(double lifetime);

    /*
        Diffuse particle emission rates.

        The diffuse particle simulator spawns particle emitters in areas
        where the fluid is likely to be aerated such as at wavecrests and
        in areas of high turbulence. The number of emitters spawned in an
        area is proportional to how sharp a wavecrest is, how turbulent
        the fluid is at a location, and how many MarkerParticles are in
        the simulation.
        
        The number of particles generated by an emitter is controlled by two 
        rates: wavecrest emission rate, and turbulence emission rate. An 
        emission rate is the number of particles generated by an emitter 
        per second. The wavecrest emission rate controls how many particles
        are generated by wavecrest emitters. The turbulence emission rate
        controls how many particles are generated by turbulence emitters.

        An important note to make about emission rates is that the number
        of particles generated scales as the simulator dimensions scale. 
        This means that a simulation with dimensions 128x128x128 will 
        generate about eight times as many diffuse particles than a
        simulation with a dimension of 64x64x64 when using the same rate
        values.
    */
    double getDiffuseParticleWavecrestEmissionRate();
    void setDiffuseParticleWavecrestEmissionRate(double r);
    double getDiffuseParticleTurbulenceEmissionRate();
    void setDiffuseParticleTurbulenceEmissionRate(double r);
    void getDiffuseParticleEmissionRates(double *rwc, double *rt);
    void setDiffuseParticleEmissionRates(double r);
    void setDiffuseParticleEmissionRates(double rwc, double rt);

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
        Enable/disable use of OpenCL for particle advection.

        Enabled by default.
    */
    void enableOpenCLParticleAdvection();
    void disableOpenCLParticleAdvection();
    bool isOpenCLParticleAdvectionEnabled();

    /*
        Enable/disable use of OpenCL for scalar fields.

        Enabled by default.
    */
    void enableOpenCLScalarField();
    void disableOpenCLScalarField();
    bool isOpenCLScalarFieldEnabled();

    /*
        Maximum workload size for the ParticleAdvector OpenCL kernel
    */
    int getParticleAdvectionKernelWorkLoadSize();
    void setParticleAdvectionKernelWorkLoadSize(int n);


    /*
        Maximum workload size for the CLScalarField OpenCL kernels
    */
    int getScalarFieldKernelWorkLoadSize();
    void setScalarFieldKernelWorkLoadSize(int n);

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
    vmath::vec3 getConstantBodyForce();
    vmath::vec3 getVariableBodyForce(double x, double y, double z);
    vmath::vec3 getVariableBodyForce(vmath::vec3 p);
    vmath::vec3 getTotalBodyForce(double x, double y, double z);
    vmath::vec3 getTotalBodyForce(vmath::vec3 p);

    /*
        Remove all added body forces.
    */
    void resetBodyForce();

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
        Add a spherical shaped fluid source to the fluid domain. 
        See sphericalfluidsource.h header for more information.
    */
    void addSphericalFluidSource(SphericalFluidSource *source);

    /*
        Add a cuboid shaped fluid source to the fluid domain. 
        See cuboidfluidsource.h header for more information.
        See examples/exaple_inflow_outflow.h for example usage.
    */
    void addCuboidFluidSource(CuboidFluidSource *source);

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
    void addSolidCells(std::vector<GridIndex> &indices);

    /*
        Remove solid cells from the simulation grid. When a solid cell is
        removed, the material will be replaced by air.

        The bordering cells of the simulation grid are permanently set as
        solid cells and will not be removed.
    */
    void removeSolidCells(std::vector<GridIndex> &indices);

    /*
        Add fluid cells to the simulation grid. Fluid cells will only be
        added if the current cell material is of type air.
    */
    void addFluidCells(std::vector<GridIndex> &indices);
    void addFluidCells(std::vector<GridIndex> &indices, vmath::vec3 velocity);
    void addFluidCells(std::vector<GridIndex> &indices, 
                       double vx, double vy, double vz);

    /*
        Remove fluid cells from the simulation grid. When a fluid cell is
        removed, all marker particles within the fluid cell will be removed
        and the material will be replaced by air.
    */
    void removeFluidCells(std::vector<GridIndex> &indices);

    /*
        Returns the number of marker particles in the simulation. Marker particles
        track where the fluid is and carry velocity data.
    */
    unsigned int getNumMarkerParticles();

    /*
        Returns a vector of all marker particles in the simulation. Marker
        particles store position and velocity vectors.
    */
    std::vector<MarkerParticle> getMarkerParticles();
    std::vector<MarkerParticle> getMarkerParticles(int startidx, int endidx);

    /*
        Returns a vector of marker particle positions. If range indices
        are specified, the vector will contain positions ranging from 
        [startidx, endidx).
    */
    std::vector<vmath::vec3> getMarkerParticlePositions();
    std::vector<vmath::vec3> getMarkerParticlePositions(int startidx, int endidx);

    /*
        Returns a vector of marker particle velocities. If range indices
        are specified, the vector will contain velocities ranging from 
        [startidx, endidx).
    */
    std::vector<vmath::vec3> getMarkerParticleVelocities();
    std::vector<vmath::vec3> getMarkerParticleVelocities(int startidx, int endidx);

    /*
        Returns the number of diffuse particles in the simulation.
    */
    unsigned int getNumDiffuseParticles();

    /*
        Returns a vector of all diffuse particles in the simulation. Diffuse particles
        store a position, velocity, lifetime, and and type (bubble, spray,
        or foam).
    */
    std::vector<DiffuseParticle> getDiffuseParticles();
    std::vector<DiffuseParticle> getDiffuseParticles(int startidx, int endidx);

    /*
        Returns a vector of diffuse particle positions. If range indices
        are specified, the vector will contain positions ranging from 
        [startidx, endidx).
    */
    std::vector<vmath::vec3> getDiffuseParticlePositions();
    std::vector<vmath::vec3> getDiffuseParticlePositions(int startidx, int endidx);

    /*
        Returns a vector of diffuse particle velocities. If range indices
        are specified, the vector will contain velocities ranging from 
        [startidx, endidx).
    */
    std::vector<vmath::vec3> getDiffuseParticleVelocities();
    std::vector<vmath::vec3> getDiffuseParticleVelocities(int startidx, int endidx);

    /*
        Returns a vector of diffuse particle lifetimes. If range indices
        are specified, the vector will contain remaining lifetimes (in seconds) 
        ranging from [startidx, endidx).
    */
    std::vector<float> getDiffuseParticleLifetimes();
    std::vector<float> getDiffuseParticleLifetimes(int startidx, int endidx);

    /*
        Returns a vector of diffuse particle types. If range indices
        are specified, the vector will contain types ranging from 
        [startidx, endidx).

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

    struct GridCellGroup {
        GridIndexVector indices;
        vmath::vec3 velocity;

        GridCellGroup() {}
        GridCellGroup(int isize, int jsize, int ksize) : 
                        indices(isize, jsize, ksize) {}
        GridCellGroup(int isize, int jsize, int ksize, vmath::vec3 v) : 
                        indices(isize, jsize, ksize),
                        velocity(v) {}
    };

    /*
        Initializing the Fluid Simulator

        Before the simulator can begin advancing the state of of the simulation,
        a call to the initialize() function must be made to initialize the fluid
        simulator.

        The initialize stage involves setting the solid boundaries of the 
        simulation domain, adding fluid cells and particles to the simulator,
        and initializing objects that use the OpenCL library.

        The fluid simulator may be initialized from a FluidSimulationSaveState object.
        In this case, solid cells, particles, and simulation dimensions are read 
        from the save state and initialized in this stage.

        The solid boundaries are initialized by setting the border of the
        FluidMaterialGrid to the solid state. The solid boundary cells must
        not be changed or removed to ensure that the simulation runs correctly.

        Fluid cells and MarkerParticles are added to the domain according to a scalar
        field of values. The scalar field is manipulated when a user adds implicit
        fluid points or fluid cuboids by making calls to the addImplicitFluidPoint()
        and addFluidCuboid() methods. In the fluid cell/particle initialization stage,
        The scalar field is converted to a triangle mesh representing the fluid surface,
        and cells that are inside and lay on the fluid surface are considered to be fluid cells.
        Each fluid cell is initialized with up to eight MarkerParticles aligned to a 2x2x2
        subgrid with a random jitter added to their positions.

        The final initialization stage is to initialize objects that use the OpenCL
        library.
    */
    void _initializeLogFile();
    void _initializeSimulationGrids(int isize, int jsize, int ksize, double dx);
    void _initializeSimulationVectors(int isize, int jsize, int ksize);
    void _initializeSimulation();
    void _logOpenCLInfo();
    void _initializeSolidCells();
    void _initializeFluidMaterial();
    void _calculateInitialFluidSurfaceScalarField(ScalarField &field);
    void _getInitialFluidCellsFromScalarField(ScalarField &field,
                                              GridIndexVector &fluidCells);
    void _getFullAndPartiallyFullFluidCells(GridIndexVector &fluidCells,
                                            GridIndexVector &fullFluidCells,
                                            GridIndexVector &partialFluidCells);
    void _getPartiallyFullFluidCellParticles(GridIndexVector &partialFluidCells,
                                             ScalarField &field,
                                             std::vector<vmath::vec3> &partialParticles);
    void _initializeMarkerParticles(GridIndexVector &fullFluidCells,
                                    std::vector<vmath::vec3> &partialParticles);
    void _initializeFluidCellIndices();
    void _initializeMarkerParticleRadius();
    double _getMarkerParticleJitter();
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

    /*
        Advancing the State of the Fluid Simulation

        A call to the update() method advances the simulation state forward
        by a specified period of time. The _stepFluid() method runs the fluid
        simulation algorithm to advance the simulation by some period of time. To 
        advance the simulation, the update() method may need to split the period 
        of time into multiple timesteps and calls to _stepFluid() in order to 
        ensure that the simulation runs in a stable manner.

        The timestep value supplied to _stepFluid() is calculated such that no
        MarkerParticle moves more than some maximum number of gridcells during
        the time step. The maximum number of cells a MarkerParticle can move is
        contained in the _CFLConditionNumber variable.

        The Fluid Simulation Algorithm:
            1.  Update fluid material
            2.  Reconstruct internal fluid surface
            3.  Compute levelset signed distance field
            4.  Reconstruct output fluid surface
            5.  Advect velocity field
            6.  Apply body forces
            7.  Pressure solve
            8.  Apply pressure
            9.  Extrapolate velocity field
            10. Update diffuse particle simulation
            11. Update MarkerParticle velocities
            12. Advance MarkerParticles
    */
    double _calculateNextTimeStep();
    double _getMaximumMarkerParticleSpeed();
    void _autosave();
    void _stepFluid(double dt);

    /*
        1. Update Fluid Material

        This is the first step of the fluid simulation algorithm. At the
        end of the last time step, the MarkerParticle positions were
        advanced through the velocity field, and now the fluid cells
        marked on the FluidMaterialGrid must be updated. A cell is
        considered to be a fluid cell if it contains at least one
        MarkerParticle.

        The simulator may contain FluidSource objects that add/remove
        particles from the simulation. FluidSource objects are updated
        in this stage. Inflow sources add new MarkerParticles to the
        domain and outflow sources remove MarkerParticles and 
        DiffuseParticles from the domain.
    */
    void _updateFluidCells();
    void _removeParticlesInSolidCells();
    void _removeMarkerParticlesInSolidCells();
    void _removeDiffuseParticlesInSolidCells();
    void _updateAddedFluidCellQueue();
    void _updateRemovedFluidCellQueue();
    void _updateFluidSources();
    void _updateInflowFluidSource(FluidSource *source);
    void _addNewFluidCells(GridIndexVector &cells, vmath::vec3 velocity);
    void _addNewFluidParticles(std::vector<vmath::vec3> &particles, vmath::vec3 velocity);
    void _getNewFluidParticles(FluidSource *source, std::vector<vmath::vec3> &particles);
    void _removeMarkerParticlesFromCells(Array3d<bool> &isRemovalCell);
    void _removeDiffuseParticlesFromCells(Array3d<bool> &isRemovalCell);

    /*
        2. Reconstruct Internal Fluid Surface

        This step of the fluid simulation algorithm reconstructs the
        surface of the fluid by converting the MarkerParticles to a
        triangle mesh. The fluid surface generated in this step is
        for internal use by the algorithm and may not be computed if
        it is not needed.

        The surface is meshed using the IsotropicParticleMesher class 
        on a grid with dimensions the same size as the simulator.
    */
    bool _isInternalFluidSurfaceNeeded();
    void _reconstructInternalFluidSurface();
    void _polygonizeInternalSurface(TriangleMesh &surface, TriangleMesh &preview);

    /*
        3. Compute LevelSet Signed Distance Field

        This step of the fluid simulation algorithm computes a signed
        distance field based on the previously computed internal surface
        mesh using the LevelSet class. A signed distance field is a 
        function that returns the distance to a surface given a point 
        in 3D space.

        The LevelSet signed distance field is used for some output
        surface reconstruction methods and in the diffuse particle
        simulation, and will not be computed if it is not needed.
        Reconstruction of the internal fluid surface is a prerequisite
        for computing the signed distance field.

        The LevelSet signed distance field uses the convention that a
        point inside the surface will have a positive distance value 
        and a point outside the surface will have a negative distance
        value.
    */
    bool _isLevelSetNeeded();
    void _updateLevelSetSignedDistanceField();

    /*
        4.  Reconstruct Output Fluid Surface

        This step of the fluid simulation algorithm prepares meshes
        and data to be output to disk. The form of data output is
        configured by the user and may include isotropic/anisotropic
        reconstructed surface meshes, 'LEGO' brick meshes and data,
        and diffuse particle meshes.

        Some forms of output have computation of the internal fluid
        surface and/or computation of the level set signed distance
        field as a prerequisite.

        If the update() method requires multiple calls to _stepFluid(),
        this stage of the algorithm will only be computed on the first
        call to _stepFluid.
    */
    void _reconstructOutputFluidSurface(double dt);
    void _outputIsotropicSurfaceMesh();
    void _outputAnisotropicSurfaceMesh();
    void _outputDiffuseMaterial();
    void _outputBrickMesh(double dt);
    std::string _numberToString(int number);
    std::string _getFrameString(int number);
    void _writeDiffuseMaterialToFile(std::string bubblefile,
                                     std::string foamfile,
                                     std::string sprayfile);
    void _writeDiffuseMaterialToFile(std::string diffusefile);
    void _writeBrickColorListToFile(TriangleMesh &mesh, std::string filename);
    void _writeBrickTextureToFile(TriangleMesh &mesh, std::string filename);
    void _writeBrickMaterialToFile(std::string brickfile, 
                                   std::string colorfile, 
                                   std::string texturefile);
    void _writeTriangleMeshToFile(TriangleMesh &mesh, std::string filename);
    void _smoothSurfaceMesh(TriangleMesh &mesh);
    void _getSmoothVertices(TriangleMesh &mesh, std::vector<int> &smoothVertices);
    bool _isVertexNearSolid(vmath::vec3 v, double eps);
    void _polygonizeIsotropicOutputSurface(TriangleMesh &surface, 
                                           TriangleMesh &preview);
    TriangleMesh _polygonizeAnisotropicOutputSurface();
    void _updateBrickGrid(double dt);

    /*
        5.  Advect Velocity Field

        This step of the fluid simulation algorithm initializes the 
        MACVelocityField for the current timestep. Velocity advection
        involves evaluating the material derivative Dq/Dt where the quantity
        q is the velocity. Since the last timestep, the MarkerParticles
        have been advanced through the velocity field, and since the
        particles carry velocity information evaluated at a time before 
        being advanced through the field, the material derivative can be 
        estimated by transferring the particle velocities onto the 
        MACVelocityField.
    */
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

    /*
        6. Apply Body Forces

        This step of the fluid simulation algorithm adds body forces,
        such as gravity, to the previously initialized MACVelocityField.

        At the end of this stage, the current state of the MACVelocityField
        is saved since it is used in a later step to update MarkerParticle
        velocities.
    */
    void _applyBodyForcesToVelocityField(double dt);
    vmath::vec3 _getConstantBodyForce();
    vmath::vec3 _getVariableBodyForce(double px, double py, double pz);
    vmath::vec3 _getVariableBodyForce(vmath::vec3 p);
    void _applyConstantBodyForces(double dt);
    void _applyVariableBodyForces(double dt);
    void _applyVariableBodyForce(vmath::vec3 (*fieldFunction)(vmath::vec3),
                                 double dt);

    /*
        7. Pressure Solve

        This step of the fluid simulation algorithm solves for a pressure
        grid, such that when applied to the MACVelocityField, results in a
        divergence-free velocity field. A divergence-free velocity field is
        required to satisfy the compressibility condition of the incompressible
        Navier-Stokes equation that this fluid simulator models. The velocity
        field is divergence-free if for each cell, the sum of all velocities
        at the cell faces is zero.

        The pressure solver uses the iterative Modified Incomplete Cholesky
        Conjugate Gradient Level 0 (MICCG(0)) algorithm to solve a sparse
        linear system for the pressure grid.
    */
    void _updatePressureGrid(Array3d<float> &pressureGrid, double dt);

    /*
        8. Apply Pressure

        This step of the fluid simulation algorithm applies the previously
        computed pressures to the MACVelocityField so that the velocity
        field is divergence-free.
    */
    void _applyPressureToVelocityField(Array3d<float> &pressureGrid, double dt);
    void _applyPressureToFaceU(int i, int j, int k, Array3d<float> &pressureGrid,
                                                    MACVelocityField &tempMACVelocity, double dt);
    void _applyPressureToFaceV(int i, int j, int k, Array3d<float> &pressureGrid,
                                                    MACVelocityField &tempMACVelocity, double dt);
    void _applyPressureToFaceW(int i, int j, int k, Array3d<float> &pressureGrid,
                                                    MACVelocityField &tempMACVelocity, double dt);
    void _commitTemporaryVelocityFieldValues(MACVelocityField &tempMACVelocity);

    /*
        9. Extrapolate Velocity Field

        This step of the fluid simulation algorithm extrapolates the
        MACVelocityField values to regions outside of the fluid. The
        velocity field must be extrapolated because in later steps, when
        advancing particles through the velocity field, velocity values
        may need to be sampled at points outside of the fluid. When a
        velocity is sampled outside of the fluid region, we want the
        value to be computed as if the sample point is inside the 
        nearest fluid region.
    */
    void _extrapolateFluidVelocities(MACVelocityField &MACGrid);

    /*
        10. Update Diffuse Particle Simulation

        This step of the fluid simulation algorithm updates the diffuse
        particle simulation if this feature is enabled. The diffuse particle
        simulation generates bubble/foam/spray particles in regions where the
        fluid would likely be aerated. The fluid is likely to be aerated at
        wavecrests where air mixes with the liquid, and in areas of high
        turbulence where air can be pulled into the liquid.

        This stage requires that the LevelSet signed distance field be
        computed as a prerequisite.
    */
    void _updateDiffuseMaterial(double dt);

    /*
        11. Update MarkerParticle Velocities

        This step of the fluid simulation algorithm updates the velocity
        quantity carried by MarkerParticles. This velocity quantity is used
        in the next time step during the velocity advection stage.

        Velocities are transferred from the grid to the particles by mixing
        the Particle In Cell (PIC) velocity update method and Fluid Implicit
        Particle (FLIP) velocity update method. The mix ratio is stored in the 
        _ratioPICFLIP variable which is a ratio of PIC to FLIP. Velocities are
        sampled on the grid via tricubic interpolations.

        The MACVelocityField that was saved after applying body forces
        is used to compute FLIP velocities.
    */
    void _updateMarkerParticleVelocities();
    void _updateRangeOfMarkerParticleVelocities(int startIdx, int endIdx);

    /*
        12. Advance MarkerParticles

        This step of the fluid simulation algorithm is the last step and
        advances the MarkerParticles through the MACVelocityField for the 
        current timestep. The Fourth-Order Runge-Kutta method of integration
        with tricubic interpolated sampling is used to update the MarkerParticle 
        positions.

        This stage also removes MarkerParticles from the domain so that the number 
        of particles in a single grid cell does not exceed a maximum stored in the
        _maxMarkerParticlesPerCell variable.
    */
    void _advanceMarkerParticles(double dt);
    void _advanceRangeOfMarkerParticles(int startIdx, int endIdx, double dt);
    vmath::vec3 _resolveParticleSolidCellCollision(vmath::vec3 p0, vmath::vec3 p1);
    void _removeMarkerParticles();
    void _shuffleMarkerParticleOrder();

    template<class T>
    void _removeItemsFromVector(FragmentedVector<T> &items, std::vector<bool> &isRemoved) {
        FLUIDSIM_ASSERT(items.size() == isRemoved.size());

        int currentidx = 0;
        for (unsigned int i = 0; i < items.size(); i++) {
            if (!isRemoved[i]) {
                items[currentidx] = items[i];
                currentidx++;
            }
        }

        int numRemoved = items.size() - currentidx;
        for (int i = 0; i < numRemoved; i++) {
            items.pop_back();
        }
        items.shrink_to_fit();
    }

    inline double _randomDouble(double min, double max) {
        return min + (double)rand() / ((double)RAND_MAX / (max - min));
    }

    template<class T>
    std::string _toString(T item) {
        std::ostringstream sstream;
        sstream << item;

        return sstream.str();
    }

    // Simulator grid dimensions and cell size
    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    // Initialization
    std::vector<FluidPoint> _fluidPoints;
    std::vector<FluidCuboid> _fluidCuboids;
    int _loadStateReadChunkSize = 50000;
    bool _isSimulationInitialized = false;

    // Update
    int _currentFrame = 0;
    int _currentTimeStep = 0;
    double _currentFrameTimeStep = 0.0;
    double _simulationTime = 0;
    double _realTime = 0;
    bool _isCurrentFrameFinished = true;
    bool _isFirstTimeStepForFrame = false;
    double _CFLConditionNumber = 5.0;
    bool _isAutosaveEnabled = true;
    LogFile _logfile;

    // Update fluid material
    FluidMaterialGrid _materialGrid;
    std::vector<FluidSource*> _fluidSources;
    std::vector<SphericalFluidSource*> _sphericalFluidSources;
    std::vector<CuboidFluidSource*> _cuboidFluidSources;
    FragmentedVector<MarkerParticle> _markerParticles;
    std::vector<GridCellGroup> _addedFluidCellQueue;
    std::vector<GridCellGroup> _removedFluidCellQueue;
    GridIndexVector _fluidCellIndices;
    double _markerParticleJitterFactor = 0.1;

    // Reconstruct internal fluid surface
    TriangleMesh _surfaceMesh;
    TriangleMesh _previewMesh;
    bool _isPreviewSurfaceMeshEnabled = false;
    double _previewdx = 0.0;

    // Compute levelset signed distance field
    LevelSet _levelset;

    // Reconstruct output fluid surface
    bool _isSurfaceMeshOutputEnabled = true;
    bool _isIsotropicSurfaceMeshReconstructionEnabled = true;
    bool _isAnisotropicSurfaceMeshReconstructionEnabled = false;
    bool _isDiffuseMaterialOutputEnabled = false;
    bool _isBubbleDiffuseMaterialEnabled = true;
    bool _isSprayDiffuseMaterialEnabled = true;
    bool _isFoamDiffuseMaterialEnabled = true;
    bool _isDiffuseMaterialFilesSeparated = false;
    bool _isBrickOutputEnabled = false;
    int _outputFluidSurfaceSubdivisionLevel = 1;
    int _numSurfaceReconstructionPolygonizerSlices = 1;
    double _surfaceReconstructionSmoothingValue = 0.5;
    int _surfaceReconstructionSmoothingIterations = 2;
    int _minimumSurfacePolyhedronTriangleCount = 0;
    double _markerParticleRadius = 0.0;
    double _markerParticleScale = 3.0;
    int _currentBrickMeshFrame = 0;
    int _brickMeshFrameOffset = -3;
    vmath::vec3 _domainOffset;
    TriangleMeshFormat _meshOutputFormat = TriangleMeshFormat::ply;
    FluidBrickGrid _fluidBrickGrid;

    // Advect velocity field
    int _maxParticlesPerVelocityAdvection = 5e6;

    // Apply body forces
    typedef vmath::vec3 (*FieldFunction)(vmath::vec3);
    std::vector<FieldFunction> _variableBodyForces;
    std::vector<vmath::vec3> _constantBodyForces;

    // Pressure solve
    double _density = 20.0;

    // Update diffuse particle simulation
    DiffuseParticleSimulation _diffuseMaterial;

    // Update MarkerParticle velocities
    int _maxParticlesPerPICFLIPUpdate = 10e6;
    double _ratioPICFLIP = 0.05f;
    MACVelocityField _MACVelocity;
    MACVelocityField _savedVelocityField;

    // Advance MarkerParticles
    int _maxParticlesPerParticleAdvection = 10e6;
    int _maxMarkerParticlesPerCell = 100;
    
    // OpenCL
    ParticleAdvector _particleAdvector;
    CLScalarField _scalarFieldAccelerator;

};

#endif