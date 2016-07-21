#include "../sphericalfluidsource.h"
#include "cbindings.h"
#include "vector3_c.h"
#include "aabb_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {
    EXPORTDLL SphericalFluidSource* SphericalFluidSource_new(Vector3_t cpos,
                                                             double radius,
                                                             Vector3_t cvelocity,
                                                             int *err) {
        vmath::vec3 p(cpos.x, cpos.y, cpos.z);
        vmath::vec3 v(cvelocity.x, cvelocity.y, cvelocity.z);

        SphericalFluidSource *source = nullptr;
        *err = CBindings::SUCCESS;
        try {
            source = new SphericalFluidSource(p, radius, v);
        } catch (std::exception &ex) {
            CBindings::set_error_message(ex);
            *err = CBindings::FAIL;
        }

        return source;
    }

    EXPORTDLL void SphericalFluidSource_destroy(SphericalFluidSource* obj) {
        delete obj;
    }

    EXPORTDLL double SphericalFluidSource_get_radius(SphericalFluidSource* obj, 
                                                     int *err) {
        return CBindings::safe_execute_method_ret_0param(
            obj, &SphericalFluidSource::getRadius, err
        );
    }

    EXPORTDLL void SphericalFluidSource_set_radius(SphericalFluidSource* obj, 
                                                   double radius, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &SphericalFluidSource::setRadius, radius, err
        );
    }
 
    EXPORTDLL void SphericalFluidSource_expand(SphericalFluidSource* obj, 
                                               double val, int *err) {
        CBindings::safe_execute_method_void_1param(
            obj, &SphericalFluidSource::expand, val, err
        );
    }
}
