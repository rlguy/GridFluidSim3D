#include "../fluidsimulationsavestate.h"
#include "vector3_c.h"
#include "gridindex_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {
    EXPORTDLL FluidSimulationSaveState* FluidSimulationSaveState_new(){ 
        return new FluidSimulationSaveState(); 
    }

    EXPORTDLL void FluidSimulationSaveState_destroy(FluidSimulationSaveState* obj) {
        delete obj;
    }

    EXPORTDLL void FluidSimulationSaveState_save_state(FluidSimulationSaveState* obj,
                                                       char* filename, 
                                                       FluidSimulation *fluidsim) {
        obj->saveState(std::string(filename), fluidsim);
    }

    EXPORTDLL int FluidSimulationSaveState_load_state(FluidSimulationSaveState* obj,
                                                       char* filename) {
        return obj->loadState(std::string(filename));
    }

    EXPORTDLL void FluidSimulationSaveState_close_state(FluidSimulationSaveState* obj) {
        obj->closeState();
    }

    EXPORTDLL void FluidSimulationSaveState_get_grid_dimensions(
            FluidSimulationSaveState* obj, int *i, int *j, int *k) {
        obj->getGridDimensions(i, j, k);
    }

    EXPORTDLL double FluidSimulationSaveState_get_cell_size(FluidSimulationSaveState* obj) {
        return obj->getCellSize();
    }

    EXPORTDLL int FluidSimulationSaveState_get_current_frame(FluidSimulationSaveState* obj) {
        return obj->getCurrentFrame();
    }

    EXPORTDLL int FluidSimulationSaveState_get_num_marker_particles(FluidSimulationSaveState* obj) {
        return obj->getNumMarkerParticles();
    }

    EXPORTDLL int FluidSimulationSaveState_get_num_diffuse_particles(FluidSimulationSaveState* obj) {
        return obj->getNumDiffuseParticles();
    }

    EXPORTDLL int FluidSimulationSaveState_get_num_solid_cells(FluidSimulationSaveState* obj) {
        return obj->getNumSolidCells();
    }

    EXPORTDLL void FluidSimulationSaveState_get_marker_particle_positions(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out) {

        std::vector<vmath::vec3> mps = obj->getMarkerParticlePositions(startidx,
                                                                       endidx);
        for (unsigned int i = 0; i < mps.size(); i++) {
            out[i].x = mps[i].x;
            out[i].y = mps[i].y;
            out[i].z = mps[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_marker_particle_velocities(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out) {

        std::vector<vmath::vec3> mvs = obj->getMarkerParticleVelocities(startidx,
                                                                        endidx);
        for (unsigned int i = 0; i < mvs.size(); i++) {
            out[i].x = mvs[i].x;
            out[i].y = mvs[i].y;
            out[i].z = mvs[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_positions(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out) {

        std::vector<vmath::vec3> dps = obj->getDiffuseParticlePositions(startidx,
                                                                       endidx);
        for (unsigned int i = 0; i < dps.size(); i++) {
            out[i].x = dps[i].x;
            out[i].y = dps[i].y;
            out[i].z = dps[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_velocities(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out) {

        std::vector<vmath::vec3> dvs = obj->getDiffuseParticleVelocities(startidx,
                                                                         endidx);
        for (unsigned int i = 0; i < dvs.size(); i++) {
            out[i].x = dvs[i].x;
            out[i].y = dvs[i].y;
            out[i].z = dvs[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_lifetimes(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            float *out) {

        std::vector<float> dls = obj->getDiffuseParticleLifetimes(startidx,
                                                                  endidx);
        for (unsigned int i = 0; i < dls.size(); i++) {
            out[i] = dls[i];
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_types(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            char *out) {

        std::vector<char> dts = obj->getDiffuseParticleTypes(startidx, endidx);
        for (unsigned int i = 0; i < dts.size(); i++) {
            out[i] = dts[i];
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_solid_cells(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            GridIndex_t *out) {

        std::vector<GridIndex> cells = obj->getSolidCells(startidx, endidx);
        for (unsigned int i = 0; i < cells.size(); i++) {
            out[i].i = cells[i].i;
            out[i].j = cells[i].j;
            out[i].k = cells[i].k;
        }
    }

    EXPORTDLL int FluidSimulationSaveState_is_fluid_brick_grid_enabled(
            FluidSimulationSaveState* obj) {
        return obj->isFluidBrickGridEnabled();
    }

    EXPORTDLL int FluidSimulationSaveState_is_load_state_initialized(
            FluidSimulationSaveState* obj) {
        return obj->isLoadStateInitialized();
    }

}
