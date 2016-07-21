#include "../fluidsource.h"
#include "cbindings.h"
#include "vector3_c.h"
#include "aabb_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {

    EXPORTDLL Vector3_t FluidSource_get_position(FluidSource* obj, int *err) {
        vmath::vec3 pos = CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::getPosition, err
        );
        return CBindings::to_struct(pos);
    }

    EXPORTDLL void FluidSource_set_position(FluidSource* obj, 
                                            Vector3_t cpos, int *err) {
        vmath::vec3 p = CBindings::to_class(cpos);
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSource::setPosition, p, err
        );
    }

    EXPORTDLL Vector3_t FluidSource_get_velocity(FluidSource* obj, int *err) {
        vmath::vec3 v = CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::getVelocity, err
        );
        return CBindings::to_struct(v);
    }

    EXPORTDLL void FluidSource_set_velocity(FluidSource* obj, 
                                            Vector3_t cvel, int *err) {
        vmath::vec3 v = CBindings::to_class(cvel);
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSource::setVelocity, v, err
        );
    }

    EXPORTDLL Vector3_t FluidSource_get_direction(FluidSource* obj, int *err) {
        vmath::vec3 d = CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::getDirection, err
        );
        return CBindings::to_struct(d);
    }

    EXPORTDLL void FluidSource_set_direction(FluidSource* obj, 
                                            Vector3_t cdir, int *err) {
        vmath::vec3 d = CBindings::to_class(cdir);
        CBindings::safe_execute_method_void_1param(
            obj, &FluidSource::setDirection, d, err
        );
    }

    EXPORTDLL AABB_t FluidSource_get_AABB(FluidSource* obj, int *err) {
        AABB bbox = CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::getAABB, err
        );
        return CBindings::to_struct(bbox);
    }

    EXPORTDLL int FluidSource_contains_point(FluidSource* obj, 
                                             Vector3_t cpos, int *err) {
        vmath::vec3 p = CBindings::to_class(cpos);
        return CBindings::safe_execute_method_ret_1param(
            obj, &FluidSource::containsPoint, p, err
        );
    }

    EXPORTDLL int FluidSource_is_inflow(FluidSource* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::isInflow, err
        );
    }

    EXPORTDLL int FluidSource_is_outflow(FluidSource* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::isOutflow, err
        );
    }

    EXPORTDLL void FluidSource_set_as_inflow(FluidSource* obj, int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSource::setAsInflow, err
        );
    }

    EXPORTDLL void FluidSource_set_as_outflow(FluidSource* obj, int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSource::setAsOutflow, err
        );
    }

    EXPORTDLL void FluidSource_activate(FluidSource* obj, int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSource::activate, err
        );
    }

    EXPORTDLL void FluidSource_deactivate(FluidSource* obj, int *err) {
        CBindings::safe_execute_method_void_0param(
            obj, &FluidSource::deactivate, err
        );
    }

    EXPORTDLL int FluidSource_is_active(FluidSource* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::isActive, err
        );
    }

    EXPORTDLL int FluidSource_get_id(FluidSource* obj, int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &FluidSource::getID, err
        );
    }

}
