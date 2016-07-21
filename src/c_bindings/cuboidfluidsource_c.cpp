#include "../cuboidfluidsource.h"
#include "cbindings.h"
#include "vector3_c.h"
#include "aabb_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {
    EXPORTDLL CuboidFluidSource* CuboidFluidSource_new(AABB_t cbbox,
                                                       Vector3_t cvelocity,
                                                       int *err) {
        AABB bbox(cbbox.position.x, cbbox.position.y, cbbox.position.z,
                  cbbox.width, cbbox.height, cbbox.depth);
        vmath::vec3 v(cvelocity.x, cvelocity.y, cvelocity.z);

        CuboidFluidSource *source = nullptr;
        *err = CBindings::SUCCESS;
        try {
            source = new CuboidFluidSource(bbox, v);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }

        return source;
    }

    EXPORTDLL void CuboidFluidSource_destroy(CuboidFluidSource* obj) {
        delete obj;
    }
 
    EXPORTDLL double CuboidFluidSource_get_width(CuboidFluidSource* obj, 
                                                 int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &CuboidFluidSource::getWidth, err
        );
    }

    EXPORTDLL void CuboidFluidSource_set_width(CuboidFluidSource* obj, 
                                               double width, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &CuboidFluidSource::setWidth, width, err
        );
    }

    EXPORTDLL double CuboidFluidSource_get_height(CuboidFluidSource* obj, 
                                                  int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &CuboidFluidSource::getHeight, err
        );
    }

    EXPORTDLL void CuboidFluidSource_set_height(CuboidFluidSource* obj, 
                                                double height, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &CuboidFluidSource::setHeight, height, err
        );
    }

    EXPORTDLL double CuboidFluidSource_get_depth(CuboidFluidSource* obj, 
                                                 int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &CuboidFluidSource::getDepth, err
        );
    }

    EXPORTDLL void CuboidFluidSource_set_depth(CuboidFluidSource* obj, 
                                               double depth, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &CuboidFluidSource::setDepth, depth, err
        );
    }

    EXPORTDLL void CuboidFluidSource_set_AABB(CuboidFluidSource* obj, 
                                              AABB_t cbbox, int *err) {
        AABB bbox = CBindings::to_class(cbbox);
        CBindings::safe_execute_method_void_1param(
            obj, &CuboidFluidSource::setAABB, bbox, err
        );
    }

    EXPORTDLL Vector3_t CuboidFluidSource_get_center(CuboidFluidSource* obj, 
                                                     int *err) {
        vmath::vec3 cpos = CBindings::safe_execute_method_ret_0param(
            obj, &CuboidFluidSource::getCenter, err
        );
        return CBindings::to_struct(cpos);
    }

    EXPORTDLL void CuboidFluidSource_set_center(CuboidFluidSource* obj, 
                                                Vector3_t cpos, int *err) {
        vmath::vec3 c = CBindings::to_class(cpos);
        CBindings::safe_execute_method_void_1param(
            obj, &CuboidFluidSource::setCenter, c, err
        );
    }

    EXPORTDLL void CuboidFluidSource_expand(CuboidFluidSource* obj, 
                                            double val, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &CuboidFluidSource::expand, val, err
        );
    }

}
