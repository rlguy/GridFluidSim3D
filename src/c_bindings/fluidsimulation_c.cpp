#include "../fluidsimulation.h"
#include "cbindings.h"
#include "aabb_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {
    EXPORTDLL FluidSimulation* FluidSimulation_new_from_empty(int *err) {
        FluidSimulation *fluidsim = nullptr;
        *err = CBindings::SUCCESS;
        try {
            fluidsim = new FluidSimulation();
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }

        return fluidsim;
    }

    EXPORTDLL FluidSimulation* FluidSimulation_new_from_dimensions(
            int isize, int jsize, int ksize, double dx, int *err) {

        FluidSimulation *fluidsim = nullptr;
        *err = CBindings::SUCCESS;
        try {
            fluidsim = new FluidSimulation(isize, jsize, ksize, dx);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }

        return fluidsim;
    }

    EXPORTDLL FluidSimulation* FluidSimulation_new_from_save_state(
            FluidSimulationSaveState *savestate, int *err) {

        FluidSimulation *fluidsim = nullptr;
        *err = CBindings::SUCCESS;
        try {
            fluidsim = new FluidSimulation(*savestate);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }

        return fluidsim; 
    }

    EXPORTDLL void FluidSimulation_destroy(FluidSimulation* obj) {
        delete obj;
    }

    EXPORTDLL void FluidSimulation_initialize(FluidSimulation* obj, int *err) {
        CBindings::safe_execute_method(obj, &FluidSimulation::initialize, err);
    }

    EXPORTDLL int FluidSimulation_is_initialized(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isInitialized, err
        );
    }

    EXPORTDLL void FluidSimulation_update(FluidSimulation* obj, double dt, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::update, dt, err
        );
    }

    EXPORTDLL void FluidSimulation_save_state(FluidSimulation* obj, char* filename, 
                                              int *err) {
        *err = CBindings::SUCCESS;
        try {
            obj->saveState(std::string(filename));
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL int FluidSimulation_get_current_frame(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getCurrentFrame, err
        );
    }

    EXPORTDLL int FluidSimulation_is_current_frame_finished(FluidSimulation* obj, 
                                                            int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isCurrentFrameFinished, err
        );
    }

    EXPORTDLL double FluidSimulation_get_cell_size(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getCellSize, err
        );
    }

    EXPORTDLL void FluidSimulation_get_grid_dimensions(
            FluidSimulation* obj, int *i, int *j, int *k, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::getGridDimensions, i, j, k, err
        );
    }

    EXPORTDLL int FluidSimulation_get_grid_width(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getGridWidth, err
        );
    }

    EXPORTDLL int FluidSimulation_get_grid_height(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getGridHeight, err
        );
    }

    EXPORTDLL int FluidSimulation_get_grid_depth(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getGridDepth, err
        );
    }

    EXPORTDLL void FluidSimulation_get_simulation_dimensions(
            FluidSimulation* obj, 
            double *width, double *height, double *depth, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::getSimulationDimensions, width, height, depth, err
        );
    }

    EXPORTDLL double FluidSimulation_get_simulation_width(FluidSimulation* obj, 
                                                          int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getSimulationWidth, err
        );
    }

    EXPORTDLL double FluidSimulation_get_simulation_height(FluidSimulation* obj, 
                                                           int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getSimulationHeight, err
        );
    }

    EXPORTDLL double FluidSimulation_get_simulation_depth(FluidSimulation* obj, 
                                                          int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getSimulationDepth, err
        );
    }

    EXPORTDLL double FluidSimulation_get_density(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getDensity, err
        );
    }

    EXPORTDLL void FluidSimulation_set_density(FluidSimulation* obj, 
                                               double density,
                                               int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setDensity, density, err
        );
    }

    EXPORTDLL int FluidSimulation_get_material(FluidSimulation* obj, 
                                                int i, int j, int k, int *err) {
        Material m = Material::air;
        *err = CBindings::SUCCESS;
        try {
            m = obj->getMaterial(i, j, k);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }

        return (int)m;
    }

    EXPORTDLL double FluidSimulation_get_marker_particle_scale(FluidSimulation* obj, 
                                                               int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getMarkerParticleScale, err
        );
    }

    EXPORTDLL void FluidSimulation_set_marker_particle_scale(FluidSimulation* obj, 
                                                             double scale,
                                                             int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setMarkerParticleScale, scale, err
        );
    }

    EXPORTDLL int FluidSimulation_get_surface_subdivision_level(FluidSimulation* obj, 
                                                                int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getSurfaceSubdivisionLevel, err
        );
    }

    EXPORTDLL void FluidSimulation_set_surface_subdivision_level(FluidSimulation* obj, 
                                                                 int level,
                                                                 int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setSurfaceSubdivisionLevel, level, err
        );
    }

    EXPORTDLL int FluidSimulation_get_num_polygonizer_slices(FluidSimulation* obj, 
                                                             int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getNumPolygonizerSlices, err
        );
    }

    EXPORTDLL void FluidSimulation_set_num_polygonizer_slices(FluidSimulation* obj, 
                                                              int numslices,
                                                              int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setNumPolygonizerSlices, numslices, err
        );
    }

    EXPORTDLL int FluidSimulation_get_min_polyhedron_triangle_count(FluidSimulation* obj, 
                                                                    int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getMinPolyhedronTriangleCount, err
        );
    }

    EXPORTDLL void FluidSimulation_set_min_polyhedron_triangle_count(FluidSimulation* obj, 
                                                                     int count,
                                                                     int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setMinPolyhedronTriangleCount, count, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_surface_mesh_output(FluidSimulation* obj,
                                                              int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableSurfaceMeshOutput, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_surface_mesh_output(FluidSimulation* obj,
                                                               int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableSurfaceMeshOutput, err
        );
    }

    EXPORTDLL int FluidSimulation_is_surface_mesh_output_enabled(FluidSimulation* obj,
                                                                 int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isSurfaceMeshOutputEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_isotropic_surface_reconstruction(FluidSimulation* obj,
                                                                           int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableIsotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_isotropic_surface_reconstruction(FluidSimulation* obj,
                                                                            int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableIsotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL int FluidSimulation_is_isotropic_surface_reconstruction_enabled(FluidSimulation* obj,
                                                                              int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isIsotropicSurfaceReconstructionEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_anisotropic_surface_reconstruction(FluidSimulation* obj,
                                                                             int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableAnisotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_anisotropic_surface_reconstruction(FluidSimulation* obj,
                                                                              int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableAnisotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL int FluidSimulation_is_anisotropic_surface_reconstruction_enabled(FluidSimulation* obj,
                                                                                int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isAnisotropicSurfaceReconstructionEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_diffuse_material_output(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableDiffuseMaterialOutput, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_diffuse_material_output(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableDiffuseMaterialOutput, err
        );
    }

    EXPORTDLL int FluidSimulation_is_diffuse_material_output_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isDiffuseMaterialOutputEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_bubble_diffuse_material(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableBubbleDiffuseMaterial, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_bubble_diffuse_material(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableBubbleDiffuseMaterial, err
        );
    }

    EXPORTDLL int FluidSimulation_is_bubble_diffuse_material_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isBubbleDiffuseMaterialEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_spray_diffuse_material(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableSprayDiffuseMaterial, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_spray_diffuse_material(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableSprayDiffuseMaterial, err
        );
    }

    EXPORTDLL int FluidSimulation_is_spray_diffuse_material_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isSprayDiffuseMaterialEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_foam_diffuse_material(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableFoamDiffuseMaterial, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_foam_diffuse_material(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableFoamDiffuseMaterial, err
        );
    }

    EXPORTDLL int FluidSimulation_is_foam_diffuse_material_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isFoamDiffuseMaterialEnabled, err
        );
    }

    EXPORTDLL int FluidSimulation_get_max_num_diffuse_particles(FluidSimulation* obj,
                                                                int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getMaxNumDiffuseParticles, err
        );
    }

    EXPORTDLL void FluidSimulation_set_max_num_diffuse_particles(FluidSimulation* obj,
                                                                 int n, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setMaxNumDiffuseParticles, n, err
        );
    }

    EXPORTDLL double FluidSimulation_get_diffuse_particle_wavecrest_emission_rate(
            FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getDiffuseParticleWavecrestEmissionRate, err
        );
    }

    EXPORTDLL void FluidSimulation_set_diffuse_particle_wavecrest_emission_rate(
            FluidSimulation* obj, double rate, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setDiffuseParticleWavecrestEmissionRate, rate, err
        );
    }

    EXPORTDLL double FluidSimulation_get_diffuse_particle_turbulence_emission_rate(
            FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::getDiffuseParticleTurbulenceEmissionRate, err
        );
    }

    EXPORTDLL void FluidSimulation_set_diffuse_particle_turbulence_emission_rate(
            FluidSimulation* obj, double rate, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::setDiffuseParticleTurbulenceEmissionRate, rate, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_brick_output(FluidSimulation* obj, 
                                                       double width, 
                                                       double height, 
                                                       double depth, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableBrickOutput, width, height, depth, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_brick_output(FluidSimulation* obj, 
                                                        int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableBrickOutput, err
        );
    }

    EXPORTDLL int FluidSimulation_is_brick_output_enabled(FluidSimulation* obj,
                                                          int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isBrickOutputEnabled, err
        );
    }

    EXPORTDLL AABB_t FluidSimulation_get_brick_AABB(FluidSimulation* obj, 
                                                    int *err) {
        AABB bbox;
        *err = CBindings::SUCCESS;
        try {
            bbox = obj->getBrickAABB();
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
        return CBindings::to_struct(bbox);
    }

    EXPORTDLL void FluidSimulation_enable_autosave(FluidSimulation* obj, 
                                                   int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::enableAutosave, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_autosave(FluidSimulation* obj,
                                                    int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::disableAutosave, err
        );
    }

    EXPORTDLL int FluidSimulation_is_autosave_enabled(FluidSimulation* obj,
                                                      int *err) {
        return CBindings::safe_execute_method(
            obj, &FluidSimulation::isAutosaveEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_add_body_force(FluidSimulation* obj,
                                                  double fx, double fy, double fz,
                                                  int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::addBodyForce, fx, fy, fz, err
        );
    }

    EXPORTDLL Vector3_t FluidSimulation_get_constant_body_force(FluidSimulation* obj,
                                                                int *err) {
        vmath::vec3 f = CBindings::safe_execute_method(
            obj, &FluidSimulation::getConstantBodyForce, err
        );
        return CBindings::to_struct(f);
    }

    EXPORTDLL Vector3_t FluidSimulation_get_variable_body_force(FluidSimulation* obj,
                                                                double px, 
                                                                double py, 
                                                                double pz, int *err) {
        vmath::vec3 f = CBindings::safe_execute_method(
            obj, &FluidSimulation::getVariableBodyForce, px, py, pz, err
        );
        return CBindings::to_struct(f);
    }

    EXPORTDLL Vector3_t FluidSimulation_get_total_body_force(FluidSimulation* obj,
                                                             double px, 
                                                             double py, 
                                                             double pz, int *err) {
        vmath::vec3 f = CBindings::safe_execute_method(
            obj, &FluidSimulation::getTotalBodyForce, px, py, pz, err
        );
        return CBindings::to_struct(f);
    }

    EXPORTDLL void FluidSimulation_reset_body_force(FluidSimulation* obj, int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::resetBodyForce, err
        );
    }

    EXPORTDLL void FluidSimulation_add_implicit_fluid_point(
            FluidSimulation* obj, 
            double x, double y, double z, double r, 
            int *err) {
        *err = CBindings::SUCCESS;
        try {
            obj->addImplicitFluidPoint(x, y, z, r);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL void FluidSimulation_add_fluid_cuboid(FluidSimulation* obj, 
                                                    AABB_t cbbox, int *err) {
        AABB bbox = CBindings::to_class(cbbox);
        CBindings::safe_execute_method(
            obj, &FluidSimulation::addFluidCuboid, bbox, err
        );
    }

    EXPORTDLL void FluidSimulation_add_spherical_fluid_source(FluidSimulation* obj, 
                                                              SphericalFluidSource *source,
                                                              int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::addSphericalFluidSource, source, err
        );
    }

    EXPORTDLL void FluidSimulation_add_cuboid_fluid_source(FluidSimulation* obj, 
                                                           CuboidFluidSource *source,
                                                           int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::addCuboidFluidSource, source, err
        );
    }

    EXPORTDLL void FluidSimulation_remove_fluid_source(FluidSimulation* obj,
                                                       FluidSource *source,
                                                       int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::removeFluidSource, source, err
        );
    }

    EXPORTDLL void FluidSimulation_remove_fluid_sources(FluidSimulation* obj, 
                                                        int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::removeFluidSources, err
        );
    }

    EXPORTDLL void FluidSimulation_add_solid_cell(FluidSimulation* obj, 
                                                  int i, int j, int k,
                                                  int *err) {
        CBindings::safe_execute_method(
            obj, &FluidSimulation::addSolidCell, i, j, k, err
        );
    }

    EXPORTDLL char* FluidSimulation_get_error_message() {
        return CBindings::get_error_message();
    }
}
