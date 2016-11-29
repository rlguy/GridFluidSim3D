#include "../fluidsimulation.h"
#include "cbindings.h"
#include "aabb_c.h"
#include "gridindex_c.h"
#include "vector3_c.h"
#include "markerparticle_c.h"
#include "diffuseparticle_c.h"

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
        CBindings::safe_execute_method_void_0param(obj, &FluidSimulation::initialize, err);
    }

    EXPORTDLL int FluidSimulation_is_initialized(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isInitialized, err
        );
    }

    EXPORTDLL void FluidSimulation_update(FluidSimulation* obj, double dt, int *err) {
        CBindings::safe_execute_method_void_1param(
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
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getCurrentFrame, err
        );
    }

    EXPORTDLL int FluidSimulation_is_current_frame_finished(FluidSimulation* obj, 
                                                            int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isCurrentFrameFinished, err
        );
    }

    EXPORTDLL double FluidSimulation_get_cell_size(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getCellSize, err
        );
    }

    EXPORTDLL void FluidSimulation_get_grid_dimensions(
            FluidSimulation* obj, int *i, int *j, int *k, int *err) {
        CBindings::safe_execute_method_void_3param(
            obj, &FluidSimulation::getGridDimensions, i, j, k, err
        );
    }

    EXPORTDLL int FluidSimulation_get_grid_width(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getGridWidth, err
        );
    }

    EXPORTDLL int FluidSimulation_get_grid_height(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getGridHeight, err
        );
    }

    EXPORTDLL int FluidSimulation_get_grid_depth(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getGridDepth, err
        );
    }

    EXPORTDLL void FluidSimulation_get_simulation_dimensions(
            FluidSimulation* obj, 
            double *width, double *height, double *depth, int *err) {
        CBindings::safe_execute_method_void_3param(
            obj, &FluidSimulation::getSimulationDimensions, width, height, depth, err
        );
    }

    EXPORTDLL double FluidSimulation_get_simulation_width(FluidSimulation* obj, 
                                                          int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getSimulationWidth, err
        );
    }

    EXPORTDLL double FluidSimulation_get_simulation_height(FluidSimulation* obj, 
                                                           int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getSimulationHeight, err
        );
    }

    EXPORTDLL double FluidSimulation_get_simulation_depth(FluidSimulation* obj, 
                                                          int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getSimulationDepth, err
        );
    }

    EXPORTDLL double FluidSimulation_get_density(FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getDensity, err
        );
    }

    EXPORTDLL void FluidSimulation_set_density(FluidSimulation* obj, 
                                               double density,
                                               int *err) {
        CBindings::safe_execute_method_void_1param(
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
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getMarkerParticleScale, err
        );
    }

    EXPORTDLL void FluidSimulation_set_marker_particle_scale(FluidSimulation* obj, 
                                                             double scale,
                                                             int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setMarkerParticleScale, scale, err
        );
    }

    EXPORTDLL int FluidSimulation_get_surface_subdivision_level(FluidSimulation* obj, 
                                                                int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getSurfaceSubdivisionLevel, err
        );
    }

    EXPORTDLL void FluidSimulation_set_surface_subdivision_level(FluidSimulation* obj, 
                                                                 int level,
                                                                 int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setSurfaceSubdivisionLevel, level, err
        );
    }

    EXPORTDLL int FluidSimulation_get_num_polygonizer_slices(FluidSimulation* obj, 
                                                             int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getNumPolygonizerSlices, err
        );
    }

    EXPORTDLL void FluidSimulation_set_num_polygonizer_slices(FluidSimulation* obj, 
                                                              int numslices,
                                                              int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setNumPolygonizerSlices, numslices, err
        );
    }

    EXPORTDLL int FluidSimulation_get_min_polyhedron_triangle_count(FluidSimulation* obj, 
                                                                    int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getMinPolyhedronTriangleCount, err
        );
    }

    EXPORTDLL void FluidSimulation_set_min_polyhedron_triangle_count(FluidSimulation* obj, 
                                                                     int count,
                                                                     int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setMinPolyhedronTriangleCount, count, err
        );
    }

    EXPORTDLL Vector3_t FluidSimulation_get_domain_offset(FluidSimulation* obj,
                                                          int *err) {
        vmath::vec3 offset = CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getDomainOffset, err
        );
        return CBindings::to_struct(offset);
    }

    EXPORTDLL void FluidSimulation_set_domain_offset(FluidSimulation* obj,
                                                     double x, double y, double z,
                                                     int *err) {
        CBindings::safe_execute_method_void_3param(
            obj, &FluidSimulation::setDomainOffset, x, y, z, err
        );
    }

    EXPORTDLL void FluidSimulation_set_mesh_output_format_as_ply(FluidSimulation* obj, 
                                                                 int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::setMeshOutputFormatAsPLY, err
        );
    }

    EXPORTDLL void FluidSimulation_set_mesh_output_format_as_bobj(FluidSimulation* obj, 
                                                                  int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::setMeshOutputFormatAsBOBJ, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_surface_mesh_output(FluidSimulation* obj,
                                                              int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableSurfaceMeshOutput, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_surface_mesh_output(FluidSimulation* obj,
                                                               int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableSurfaceMeshOutput, err
        );
    }

    EXPORTDLL int FluidSimulation_is_surface_mesh_output_enabled(FluidSimulation* obj,
                                                                 int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isSurfaceMeshOutputEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_isotropic_surface_reconstruction(FluidSimulation* obj,
                                                                           int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableIsotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_isotropic_surface_reconstruction(FluidSimulation* obj,
                                                                            int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableIsotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL int FluidSimulation_is_isotropic_surface_reconstruction_enabled(FluidSimulation* obj,
                                                                              int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isIsotropicSurfaceReconstructionEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_preview_mesh_output(FluidSimulation* obj,
                                                              double dx,
                                                              int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::enablePreviewMeshOutput, dx, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_preview_mesh_output(FluidSimulation* obj,
                                                               int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disablePreviewMeshOutput, err
        );
    }

    EXPORTDLL int FluidSimulation_is_preview_mesh_output_enabled(FluidSimulation* obj,
                                                                 int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isPreviewMeshOutputEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_anisotropic_surface_reconstruction(FluidSimulation* obj,
                                                                             int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableAnisotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_anisotropic_surface_reconstruction(FluidSimulation* obj,
                                                                              int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableAnisotropicSurfaceReconstruction, err
        );
    }

    EXPORTDLL int FluidSimulation_is_anisotropic_surface_reconstruction_enabled(FluidSimulation* obj,
                                                                                int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isAnisotropicSurfaceReconstructionEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_diffuse_material_output(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableDiffuseMaterialOutput, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_diffuse_material_output(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableDiffuseMaterialOutput, err
        );
    }

    EXPORTDLL int FluidSimulation_is_diffuse_material_output_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isDiffuseMaterialOutputEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_bubble_diffuse_material(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableBubbleDiffuseMaterial, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_bubble_diffuse_material(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableBubbleDiffuseMaterial, err
        );
    }

    EXPORTDLL int FluidSimulation_is_bubble_diffuse_material_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isBubbleDiffuseMaterialEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_spray_diffuse_material(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableSprayDiffuseMaterial, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_spray_diffuse_material(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableSprayDiffuseMaterial, err
        );
    }

    EXPORTDLL int FluidSimulation_is_spray_diffuse_material_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isSprayDiffuseMaterialEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_foam_diffuse_material(FluidSimulation* obj,
                                                                  int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableFoamDiffuseMaterial, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_foam_diffuse_material(FluidSimulation* obj,
                                                                   int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableFoamDiffuseMaterial, err
        );
    }

    EXPORTDLL int FluidSimulation_is_foam_diffuse_material_enabled(FluidSimulation* obj,
                                                                     int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isFoamDiffuseMaterialEnabled, err
        );
    }

    EXPORTDLL int FluidSimulation_get_max_num_diffuse_particles(FluidSimulation* obj,
                                                                int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getMaxNumDiffuseParticles, err
        );
    }

    EXPORTDLL void FluidSimulation_set_max_num_diffuse_particles(FluidSimulation* obj,
                                                                 int n, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setMaxNumDiffuseParticles, n, err
        );
    }

    EXPORTDLL double FluidSimulation_get_max_diffuse_particle_lifetime(FluidSimulation* obj,
                                                                       int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getMaxDiffuseParticleLifetime, err
        );
    }

    EXPORTDLL void FluidSimulation_set_max_diffuse_particle_lifetime(FluidSimulation* obj,
                                                                     double lifetime, 
                                                                     int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setMaxDiffuseParticleLifetime, lifetime, err
        );
    }

    EXPORTDLL double FluidSimulation_get_diffuse_particle_wavecrest_emission_rate(
            FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getDiffuseParticleWavecrestEmissionRate, err
        );
    }

    EXPORTDLL void FluidSimulation_set_diffuse_particle_wavecrest_emission_rate(
            FluidSimulation* obj, double rate, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setDiffuseParticleWavecrestEmissionRate, rate, err
        );
    }

    EXPORTDLL double FluidSimulation_get_diffuse_particle_turbulence_emission_rate(
            FluidSimulation* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getDiffuseParticleTurbulenceEmissionRate, err
        );
    }

    EXPORTDLL void FluidSimulation_set_diffuse_particle_turbulence_emission_rate(
            FluidSimulation* obj, double rate, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setDiffuseParticleTurbulenceEmissionRate, rate, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_brick_output(FluidSimulation* obj, 
                                                       double width, 
                                                       double height, 
                                                       double depth, int *err) {
        CBindings::safe_execute_method_void_3param(
            obj, &FluidSimulation::enableBrickOutput, width, height, depth, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_brick_output(FluidSimulation* obj, 
                                                        int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableBrickOutput, err
        );
    }

    EXPORTDLL int FluidSimulation_is_brick_output_enabled(FluidSimulation* obj,
                                                          int *err) {
        return CBindings::safe_execute_method_ret_0param(
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
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableAutosave, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_autosave(FluidSimulation* obj,
                                                    int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableAutosave, err
        );
    }

    EXPORTDLL int FluidSimulation_is_autosave_enabled(FluidSimulation* obj,
                                                      int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isAutosaveEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_opencl_particle_advection(FluidSimulation* obj, 
                                                   int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableOpenCLParticleAdvection, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_opencl_particle_advection(FluidSimulation* obj,
                                                    int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableOpenCLParticleAdvection, err
        );
    }

    EXPORTDLL int FluidSimulation_is_opencl_particle_advection_enabled(FluidSimulation* obj,
                                                      int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isOpenCLParticleAdvectionEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_enable_opencl_scalar_field(FluidSimulation* obj, 
                                                   int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::enableOpenCLScalarField, err
        );
    }

    EXPORTDLL int FluidSimulation_get_particle_advection_kernel_workload_size(
            FluidSimulation* obj, int *err) {

        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getParticleAdvectionKernelWorkLoadSize, err
        );
    }

    EXPORTDLL void FluidSimulation_set_particle_advection_kernel_workload_size(
            FluidSimulation* obj, int size, int *err) {

        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setParticleAdvectionKernelWorkLoadSize, size, err
        );
    }

    EXPORTDLL int FluidSimulation_get_scalar_field_kernel_workload_size(
            FluidSimulation* obj, int *err) {

        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getScalarFieldKernelWorkLoadSize, err
        );
    }

    EXPORTDLL void FluidSimulation_set_scalar_field_kernel_workload_size(
            FluidSimulation* obj, int size, int *err) {
        
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::setScalarFieldKernelWorkLoadSize, size, err
        );
    }

    EXPORTDLL void FluidSimulation_disable_opencl_scalar_field(FluidSimulation* obj,
                                                    int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::disableOpenCLScalarField, err
        );
    }

    EXPORTDLL int FluidSimulation_is_opencl_scalar_field_enabled(FluidSimulation* obj,
                                                      int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::isOpenCLScalarFieldEnabled, err
        );
    }

    EXPORTDLL void FluidSimulation_add_body_force(FluidSimulation* obj,
                                                  double fx, double fy, double fz,
                                                  int *err) {
        CBindings::safe_execute_method_void_3param(
            obj, &FluidSimulation::addBodyForce, fx, fy, fz, err
        );
    }

    EXPORTDLL Vector3_t FluidSimulation_get_constant_body_force(FluidSimulation* obj,
                                                                int *err) {
        vmath::vec3 f = CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getConstantBodyForce, err
        );
        return CBindings::to_struct(f);
    }

    EXPORTDLL Vector3_t FluidSimulation_get_variable_body_force(FluidSimulation* obj,
                                                                double px, 
                                                                double py, 
                                                                double pz, int *err) {
        vmath::vec3 f = CBindings::safe_execute_method_ret_3param(
            obj, &FluidSimulation::getVariableBodyForce, px, py, pz, err
        );
        return CBindings::to_struct(f);
    }

    EXPORTDLL Vector3_t FluidSimulation_get_total_body_force(FluidSimulation* obj,
                                                             double px, 
                                                             double py, 
                                                             double pz, int *err) {
        vmath::vec3 f = CBindings::safe_execute_method_ret_3param(
            obj, &FluidSimulation::getTotalBodyForce, px, py, pz, err
        );
        return CBindings::to_struct(f);
    }

    EXPORTDLL void FluidSimulation_reset_body_force(FluidSimulation* obj, int *err) {
        CBindings::safe_execute_method_void_0param(
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
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::addFluidCuboid, bbox, err
        );
    }

    EXPORTDLL void FluidSimulation_add_spherical_fluid_source(FluidSimulation* obj, 
                                                              SphericalFluidSource *source,
                                                              int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::addSphericalFluidSource, source, err
        );
    }

    EXPORTDLL void FluidSimulation_add_cuboid_fluid_source(FluidSimulation* obj, 
                                                           CuboidFluidSource *source,
                                                           int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::addCuboidFluidSource, source, err
        );
    }

    EXPORTDLL void FluidSimulation_remove_fluid_source(FluidSimulation* obj,
                                                       FluidSource *source,
                                                       int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSimulation::removeFluidSource, source, err
        );
    }

    EXPORTDLL void FluidSimulation_remove_fluid_sources(FluidSimulation* obj, 
                                                        int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSimulation::removeFluidSources, err
        );
    }

    EXPORTDLL void FluidSimulation_add_solid_cells(FluidSimulation* obj, 
                                                   GridIndex_t *cells,
                                                   int n,
                                                   int *err) {
        std::vector<GridIndex> indices;
        indices.reserve(n);
        for (int i = 0; i < n; i++) {
            indices.push_back(GridIndex(cells[i].i, cells[i].j, cells[i].k));
        }

        *err = CBindings::SUCCESS;
        try {
            obj->addSolidCells(indices);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL void FluidSimulation_remove_solid_cells(FluidSimulation* obj, 
                                                      GridIndex_t *cells,
                                                      int n,
                                                      int *err) {
        std::vector<GridIndex> indices;
        indices.reserve(n);
        for (int i = 0; i < n; i++) {
            indices.push_back(GridIndex(cells[i].i, cells[i].j, cells[i].k));
        }

        *err = CBindings::SUCCESS;
        try {
            obj->removeSolidCells(indices);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL void FluidSimulation_add_fluid_cells(FluidSimulation* obj, 
                                                   GridIndex_t *cells,
                                                   int n,
                                                   int *err) {
        std::vector<GridIndex> indices;
        indices.reserve(n);
        for (int i = 0; i < n; i++) {
            indices.push_back(GridIndex(cells[i].i, cells[i].j, cells[i].k));
        }

        *err = CBindings::SUCCESS;
        try {
            obj->addFluidCells(indices);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL void FluidSimulation_add_fluid_cells_velocity(FluidSimulation* obj, 
                                                            GridIndex_t *cells,
                                                            Vector3_t velocity,
                                                            int n,
                                                            int *err) {
        std::vector<GridIndex> indices;
        indices.reserve(n);
        for (int i = 0; i < n; i++) {
            indices.push_back(GridIndex(cells[i].i, cells[i].j, cells[i].k));
        }
        vmath::vec3 v(velocity.x, velocity.y, velocity.z);

        *err = CBindings::SUCCESS;
        try {
            obj->addFluidCells(indices, v);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL void FluidSimulation_remove_fluid_cells(FluidSimulation* obj, 
                                                      GridIndex_t *cells,
                                                      int n,
                                                      int *err) {
        std::vector<GridIndex> indices;
        indices.reserve(n);
        for (int i = 0; i < n; i++) {
            indices.push_back(GridIndex(cells[i].i, cells[i].j, cells[i].k));
        }

        *err = CBindings::SUCCESS;
        try {
            obj->removeFluidCells(indices);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }
    }

    EXPORTDLL int FluidSimulation_get_num_marker_particles(FluidSimulation* obj, 
                                                           int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getNumMarkerParticles, err
        );
    }

    EXPORTDLL void FluidSimulation_get_marker_particles(
            FluidSimulation* obj, 
            int startidx, int endidx,
            MarkerParticle_t *out, int *err) {

        std::vector<MarkerParticle> mps = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getMarkerParticles,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < mps.size(); i++) {
            out[i] = CBindings::to_struct(mps[i]);
        }
    }

    EXPORTDLL void FluidSimulation_get_marker_particle_positions(
            FluidSimulation* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> mps = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getMarkerParticlePositions,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < mps.size(); i++) {
            out[i] = CBindings::to_struct(mps[i]);
        }
    }

    EXPORTDLL void FluidSimulation_get_marker_particle_velocities(
            FluidSimulation* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> mvs = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getMarkerParticleVelocities,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < mvs.size(); i++) {
            out[i] = CBindings::to_struct(mvs[i]);
        }
    }

    EXPORTDLL int FluidSimulation_get_num_diffuse_particles(FluidSimulation* obj, 
                                                            int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSimulation::getNumDiffuseParticles, err
        );
    }

    EXPORTDLL void FluidSimulation_get_diffuse_particles(
            FluidSimulation* obj, 
            int startidx, int endidx,
            DiffuseParticle_t *out, int *err) {

        std::vector<DiffuseParticle> dps = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getDiffuseParticles,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < dps.size(); i++) {
            out[i] = CBindings::to_struct(dps[i]);
        }
    }

    EXPORTDLL void FluidSimulation_get_diffuse_particle_positions(
            FluidSimulation* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> dps = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getDiffuseParticlePositions,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < dps.size(); i++) {
            out[i] = CBindings::to_struct(dps[i]);
        }
    }

    EXPORTDLL void FluidSimulation_get_diffuse_particle_velocities(
            FluidSimulation* obj, 
            int startidx, int endidx,
            Vector3_t *out, int *err) {

        std::vector<vmath::vec3> dvs = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getDiffuseParticleVelocities,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < dvs.size(); i++) {
            out[i] = CBindings::to_struct(dvs[i]);
        }
    }

    EXPORTDLL void FluidSimulation_get_diffuse_particle_lifetimes(
            FluidSimulation* obj, 
            int startidx, int endidx,
            float *out, int *err) {

        std::vector<float> lfs = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getDiffuseParticleLifetimes,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < lfs.size(); i++) {
            out[i] = lfs[i];
        }
    }

    EXPORTDLL void FluidSimulation_get_diffuse_particle_types(
            FluidSimulation* obj, 
            int startidx, int endidx,
            char *out, int *err) {

        std::vector<char> types = CBindings::safe_execute_method_ret_2param(
            obj, &FluidSimulation::getDiffuseParticleTypes,
            startidx, endidx, err
        );

        for (unsigned int i = 0; i < types.size(); i++) {
            out[i] = types[i];
        }
    }

    EXPORTDLL char* FluidSimulation_get_error_message() {
        return CBindings::get_error_message();
    }
}
