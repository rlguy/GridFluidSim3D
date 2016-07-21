#include "../fluidsimulationsavestate.h"
#include "cbindings.h"
#include "vector3_c.h"
#include "gridindex_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {
    EXPORTDLL FluidSimulationSaveState* FluidSimulationSaveState_new(int *err){ 
        FluidSimulationSaveState *state = nullptr;
        *err = CBindings::SUCCESS;
        try {
            state = new FluidSimulationSaveState();
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }

        return state;
    }

    EXPORTDLL void FluidSimulationSaveState_destroy(FluidSimulationSaveState* obj) {
        delete obj;
    }

    EXPORTDLL void FluidSimulationSaveState_save_state(FluidSimulationSaveState* obj,
                                                       char* filename, 
                                                       FluidSimulation *fluidsim,
                                                       int *err) {
        *err = CBindings::SUCCESS;
        try {
            obj->saveState(std::string(filename), fluidsim);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL int FluidSimulationSaveState_load_state(FluidSimulationSaveState* obj,
                                                       char* filename,
                                                       int *err) {
        std::string filestr(filename);
        return CBindings::safe_execute_method_ret_1param(
            obj, &FluidSimulationSaveState::loadState, filestr, err
        );
    }

    EXPORTDLL void FluidSimulationSaveState_close_state(FluidSimulationSaveState* obj,
                                                        int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulationSaveState::closeState, err
        );
    }

    EXPORTDLL void FluidSimulationSaveState_get_grid_dimensions(
            FluidSimulationSaveState* obj, int *i, int *j, int *k, int *err) {
        CBindings::safe_execute_method_void_3param(
            obj, &FluidSimulationSaveState::getGridDimensions, i, j, k, err
        );
    }

    EXPORTDLL double FluidSimulationSaveState_get_cell_size(FluidSimulationSaveState* obj,
                                                            int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulationSaveState::getCellSize, err
        );
    }

    EXPORTDLL int FluidSimulationSaveState_get_current_frame(FluidSimulationSaveState* obj,
                                                             int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulationSaveState::getCurrentFrame, err
        );
    }

    EXPORTDLL int FluidSimulationSaveState_get_num_marker_particles(FluidSimulationSaveState* obj,
                                                                    int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulationSaveState::getNumMarkerParticles, err
        );
    }

    EXPORTDLL int FluidSimulationSaveState_get_num_diffuse_particles(FluidSimulationSaveState* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulationSaveState::getNumDiffuseParticles, err
        );
    }

    EXPORTDLL int FluidSimulationSaveState_get_num_solid_cells(FluidSimulationSaveState* obj,
                                                               int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulationSaveState::getNumSolidCells, err
        );
    }

    EXPORTDLL void FluidSimulationSaveState_get_marker_particle_positions(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> mps = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulationSaveState::getMarkerParticlePositions,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < mps.size(); i++) {
            out[i].x = mps[i].x;
            out[i].y = mps[i].y;
            out[i].z = mps[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_marker_particle_velocities(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> mvs = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulationSaveState::getMarkerParticleVelocities,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < mvs.size(); i++) {
            out[i].x = mvs[i].x;
            out[i].y = mvs[i].y;
            out[i].z = mvs[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_positions(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> dps = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulationSaveState::getDiffuseParticlePositions,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < dps.size(); i++) {
            out[i].x = dps[i].x;
            out[i].y = dps[i].y;
            out[i].z = dps[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_velocities(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> dvs = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulationSaveState::getDiffuseParticleVelocities,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < dvs.size(); i++) {
            out[i].x = dvs[i].x;
            out[i].y = dvs[i].y;
            out[i].z = dvs[i].z;
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_lifetimes(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            float *out, int *err) {

        std::vector<float> dls = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulationSaveState::getDiffuseParticleLifetimes,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < dls.size(); i++) {
            out[i] = dls[i];
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_diffuse_particle_types(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            char *out, int *err) {

        std::vector<char> dts = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulationSaveState::getDiffuseParticleTypes,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < dts.size(); i++) {
            out[i] = dts[i];
        }
    }

    EXPORTDLL void FluidSimulationSaveState_get_solid_cells(
            FluidSimulationSaveState* obj, 
            int startidx, int endidx,
            GridIndex_t *out, int *err) {

        std::vector<GridIndex> cells = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulationSaveState::getSolidCells,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < cells.size(); i++) {
            out[i].i = cells[i].i;
            out[i].j = cells[i].j;
            out[i].k = cells[i].k;
        }
    }

    EXPORTDLL int FluidSimulationSaveState_is_fluid_brick_grid_enabled(
            FluidSimulationSaveState* obj, int *err) {

        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulationSaveState::isFluidBrickGridEnabled, err
        );
    }

    EXPORTDLL int FluidSimulationSaveState_is_load_state_initialized(
            FluidSimulationSaveState* obj, int *err) {

        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulationSaveState::isLoadStateInitialized, err
        );
    }

}
