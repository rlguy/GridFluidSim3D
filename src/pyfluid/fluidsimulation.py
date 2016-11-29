import ctypes
from ctypes import c_void_p, c_char_p, c_char, c_int, c_float, c_double, byref
import numbers

from .pyfluid import pyfluid as lib
from .fluidsimulationsavestate import FluidSimulationSaveState
from .vector3 import Vector3, Vector3_t
from .gridindex import GridIndex, GridIndex_t
from .aabb import AABB, AABB_t
from . import pybindings as pb
from . import method_decorators as decorators



def _check_simulation_initialized(func):
    def check_initialized_wrapper(self, *args):
        if isinstance(self, FluidSimulation) and not self.is_initialized():
            errmsg = "FluidSimulation must be initialized before calling this method"
            raise RuntimeError(errmsg)
        return func(self, *args)

    return check_initialized_wrapper

def _check_simulation_not_initialized(func):
    def check_not_initialized_wrapper(self, *args):
        if isinstance(self, FluidSimulation) and self.is_initialized():
            errmsg = "This method must be called before FluidSimulation is initialized"
            raise RuntimeError(errmsg)
        return func(self, *args)

    return check_not_initialized_wrapper

class FluidSimulation(object):

    def __init__(self, isize = None, jsize = None, ksize = None, dx = None):
        is_empty_constructor = all(x == None for x in (isize, jsize, ksize, dx))
        is_dimensions_constructor = (isinstance(isize, int) and
                                     isinstance(jsize, int) and
                                     isinstance(ksize, int) and
                                     isinstance(dx, numbers.Real))

        if is_empty_constructor:
            self._init_from_empty()
        elif is_dimensions_constructor:
            self._init_from_dimensions(isize, jsize, ksize, dx)
        else:
            errmsg = "FluidSimulation must be initialized with types:\n"
            errmsg += "isize:\t" + (str(int) + "\n" + 
                      "jsize:\t" + str(int) + "\n" + 
                      "ksize:\t" + str(int) + "\n" + 
                      "dx:\t" + str(float))
            raise TypeError(errmsg)

    def _init_from_empty(self):
        libfunc = lib.FluidSimulation_new_from_empty
        pb.init_lib_func(libfunc, [c_void_p], c_void_p)
        self._obj = pb.execute_lib_func(libfunc, [])

    @decorators.check_gt_zero
    def _init_from_dimensions(self, isize, jsize, ksize, dx):
        libfunc = lib.FluidSimulation_new_from_dimensions
        pb.init_lib_func(libfunc, 
                            [c_int, c_int, c_int, c_double, c_void_p], 
                            c_void_p)
        self._obj = pb.execute_lib_func(libfunc, [isize, jsize, ksize, dx])

    @classmethod
    @decorators.check_type(FluidSimulationSaveState)
    def from_save_state(cls, savestate):
        self = cls()
        lib.FluidSimulation_destroy(self._obj)

        libfunc = lib.FluidSimulation_new_from_save_state
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_void_p)
        self._obj = pb.execute_lib_func(libfunc, [savestate()])
        return self

    def __del__(self):
        libfunc = lib.FluidSimulation_destroy
        pb.init_lib_func(libfunc, [c_void_p], None)
        try:
            libfunc(self._obj)
        except:
            pass

    def __call__(self):
        return self._obj

    def initialize(self):
        if self.is_initialized():
            return
        libfunc = lib.FluidSimulation_initialize
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    def is_initialized(self):
        """ KJSDSJDKJSJKDJKSDSD
           SDKLLKSDLKSDKL """
        libfunc = lib.FluidSimulation_is_initialized
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @_check_simulation_initialized
    def update(self, dt):
        libfunc = lib.FluidSimulation_update
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), dt])

    @_check_simulation_initialized
    def save_state(self, filename):
        libfunc = lib.FluidSimulation_save_state
        pb.init_lib_func(libfunc, [c_void_p, c_char_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), filename])

    def get_current_frame(self):
        libfunc = lib.FluidSimulation_get_current_frame
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    def is_current_frame_finished(self):
        libfunc = lib.FluidSimulation_is_current_frame_finished
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    def get_cell_size(self):
        libfunc = lib.FluidSimulation_get_cell_size
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    def get_grid_dimensions(self):
        libfunc = lib.FluidSimulation_get_grid_dimensions
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p], None)
        isize = c_int()
        jsize = c_int()
        ksize = c_int()
        success = c_int()
        libfunc(self(), byref(isize), byref(jsize), byref(ksize), byref(success))
        pb.check_success(success, libfunc.__name__ + " - ")

        return GridIndex(isize.value, jsize.value, ksize.value)

    def get_grid_width(self):
        libfunc = lib.FluidSimulation_get_grid_width
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    def get_grid_height(self):
        libfunc = lib.FluidSimulation_get_grid_height
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    def get_grid_depth(self):
        libfunc = lib.FluidSimulation_get_grid_depth
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    def get_simulation_dimensions(self):
        libfunc = lib.FluidSimulation_get_simulation_dimensions
        pb.init_lib_func(libfunc, 
                            [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p], 
                            None)
        width = c_double()
        height = c_double()
        depth = c_double()
        success = c_int()
        libfunc(self(), byref(width), byref(height), byref(depth), byref(success))
        pb.check_success(success, libfunc.__name__ + " - ")

        return Vector3(width.value, height.value, depth.value)

    def get_simulation_width(self):
        libfunc = lib.FluidSimulation_get_simulation_width
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    def get_simulation_height(self):
        libfunc = lib.FluidSimulation_get_simulation_height
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    def get_simulation_depth(self):
        libfunc = lib.FluidSimulation_get_simulation_depth
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @property
    def density(self):
        libfunc = lib.FluidSimulation_get_density
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @density.setter
    @decorators.check_gt_zero
    def density(self, value):
        libfunc = lib.FluidSimulation_set_density
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), value])


    @decorators.ijk_or_gridindex
    def get_material(self, i, j, k):
        libfunc = lib.FluidSimulation_get_material
        pb.init_lib_func(libfunc, [c_void_p, c_int, c_int, c_int, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self(), i, j, k])

    @property
    def marker_particle_scale(self):
        libfunc = lib.FluidSimulation_get_marker_particle_scale
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @marker_particle_scale.setter
    @decorators.check_ge_zero
    def marker_particle_scale(self, scale):
        libfunc = lib.FluidSimulation_set_marker_particle_scale
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), scale])

    @property
    def surface_subdivision_level(self):
        libfunc = lib.FluidSimulation_get_surface_subdivision_level
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @surface_subdivision_level.setter
    @decorators.check_ge(1)
    def surface_subdivision_level(self, level):
        libfunc = lib.FluidSimulation_set_surface_subdivision_level
        pb.init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), level])

    @property
    def num_polygonizer_slices(self):
        libfunc = lib.FluidSimulation_get_num_polygonizer_slices
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @num_polygonizer_slices.setter
    @decorators.check_ge(1)
    def num_polygonizer_slices(self, slices):
        libfunc = lib.FluidSimulation_set_num_polygonizer_slices
        pb.init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), slices])

    @property
    def min_polyhedron_triangle_count(self):
        libfunc = lib.FluidSimulation_get_min_polyhedron_triangle_count
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @min_polyhedron_triangle_count.setter
    @decorators.check_ge_zero
    def min_polyhedron_triangle_count(self, count):
        libfunc = lib.FluidSimulation_set_min_polyhedron_triangle_count
        pb.init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), count])

    def get_domain_offset(self):
        libfunc = lib.FluidSimulation_get_domain_offset
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], Vector3_t)
        cvect = pb.execute_lib_func(libfunc, [self()])
        return Vector3.from_struct(cvect)

    @decorators.xyz_or_vector
    def set_domain_offset(self, x, y, z):
        libfunc = lib.FluidSimulation_set_domain_offset
        pb.init_lib_func(
            libfunc, 
            [c_void_p, c_double, c_double, c_double, c_void_p], None
        )
        pb.execute_lib_func(libfunc, [self(), x, y, z])

    def set_mesh_output_format_as_ply(self):
        libfunc = lib.FluidSimulation_set_mesh_output_format_as_ply
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    def set_mesh_output_format_as_bobj(self):
        libfunc = lib.FluidSimulation_set_mesh_output_format_as_bobj
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_surface_mesh_output(self):
        libfunc = lib.FluidSimulation_is_surface_mesh_output_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_surface_mesh_output.setter
    def enable_surface_mesh_output(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_surface_mesh_output
        else:
            libfunc = lib.FluidSimulation_disable_surface_mesh_output
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_isotropic_surface_reconstruction(self):
        libfunc = lib.FluidSimulation_is_isotropic_surface_reconstruction_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_isotropic_surface_reconstruction.setter
    def enable_isotropic_surface_reconstruction(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_isotropic_surface_reconstruction
        else:
            libfunc = lib.FluidSimulation_disable_isotropic_surface_reconstruction
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_preview_mesh_output(self):
        libfunc = lib.FluidSimulation_is_preview_mesh_output_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_preview_mesh_output.setter
    @decorators.check_ge_zero
    def enable_preview_mesh_output(self, cellsize):
        if cellsize:
            libfunc = lib.FluidSimulation_enable_preview_mesh_output
            pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
            pb.execute_lib_func(libfunc, [self(), cellsize])
        else:
            libfunc = lib.FluidSimulation_disable_preview_mesh_output
            pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
            pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_anisotropic_surface_reconstruction(self):
        libfunc = lib.FluidSimulation_is_anisotropic_surface_reconstruction_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_anisotropic_surface_reconstruction.setter
    def enable_anisotropic_surface_reconstruction(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_anisotropic_surface_reconstruction
        else:
            libfunc = lib.FluidSimulation_disable_anisotropic_surface_reconstruction
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_diffuse_material_output(self):
        libfunc = lib.FluidSimulation_is_diffuse_material_output_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_diffuse_material_output.setter
    def enable_diffuse_material_output(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_diffuse_material_output
        else:
            libfunc = lib.FluidSimulation_disable_diffuse_material_output
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_bubble_diffuse_material(self):
        libfunc = lib.FluidSimulation_is_bubble_diffuse_material_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_bubble_diffuse_material.setter
    def enable_bubble_diffuse_material(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_bubble_diffuse_material
        else:
            libfunc = lib.FluidSimulation_disable_bubble_diffuse_material
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_spray_diffuse_material(self):
        libfunc = lib.FluidSimulation_is_spray_diffuse_material_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_spray_diffuse_material.setter
    def enable_spray_diffuse_material(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_spray_diffuse_material
        else:
            libfunc = lib.FluidSimulation_disable_spray_diffuse_material
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_foam_diffuse_material(self):
        libfunc = lib.FluidSimulation_is_foam_diffuse_material_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_foam_diffuse_material.setter
    def enable_foam_diffuse_material(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_get_max_num_diffuse_particles
        else:
            libfunc = lib.FluidSimulation_disable_foam_diffuse_material
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def max_num_diffuse_particles(self):
        libfunc = lib.FluidSimulation_get_max_num_diffuse_particles
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @max_num_diffuse_particles.setter
    def max_num_diffuse_particles(self, num):
        libfunc = lib.FluidSimulation_set_max_num_diffuse_particles
        pb.init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), num])

    @property
    def max_diffuse_particle_lifetime(self):
        libfunc = lib.FluidSimulation_get_max_diffuse_particle_lifetime
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @max_diffuse_particle_lifetime.setter
    def max_diffuse_particle_lifetime(self, lifetime):
        libfunc = lib.FluidSimulation_set_max_diffuse_particle_lifetime
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), lifetime])

    @property
    def diffuse_particle_wavecrest_emission_rate(self):
        libfunc = lib.FluidSimulation_get_diffuse_particle_wavecrest_emission_rate
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @diffuse_particle_wavecrest_emission_rate.setter
    @decorators.check_ge_zero
    def diffuse_particle_wavecrest_emission_rate(self, rate):
        libfunc = lib.FluidSimulation_set_diffuse_particle_wavecrest_emission_rate
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), rate])

    @property
    def diffuse_particle_turbulence_emission_rate(self):
        libfunc = lib.FluidSimulation_get_diffuse_particle_turbulence_emission_rate
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @diffuse_particle_turbulence_emission_rate.setter
    @decorators.check_ge_zero
    def diffuse_particle_turbulence_emission_rate(self, rate):
        libfunc = lib.FluidSimulation_set_diffuse_particle_turbulence_emission_rate
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), rate])

    @property
    def enable_brick_output(self):
        libfunc = lib.FluidSimulation_is_brick_output_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_brick_output.setter
    def enable_brick_output(self, params):
        errmsg = ("enable_brick_output must be enabled with an iterable in " +
                   "in the form (True, AABB) or disabled with a value of False")
        if hasattr(params, '__iter__') and len(params) >= 2:
            boolval = params[0]
            bbox = params[1]
        elif params is False:
            boolval = params
        else:
            raise TypeError(errmsg)

        if boolval:
            libfunc = lib.FluidSimulation_enable_brick_output
            pb.init_lib_func(
                libfunc, [c_void_p, c_double, c_double, c_double, c_void_p], None
            )
            try:
                w, h, d = bbox.width, bbox.height, bbox.depth
            except:
                raise TypeError(errmsg)
            pb.execute_lib_func(libfunc, [self(), w, h, d])
        else:
            libfunc = lib.FluidSimulation_disable_brick_output
            pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
            pb.execute_lib_func(libfunc, [self()])

    def get_brick_AABB(self):
        libfunc = lib.FluidSimulation_get_brick_AABB
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], AABB_t)
        cbbox = pb.execute_lib_func(libfunc, [self()])
        return AABB.from_struct(cbbox)

    @property
    def enable_autosave(self):
        libfunc = lib.FluidSimulation_is_autosave_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_autosave.setter
    def enable_autosave(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_autosave
        else:
            libfunc = lib.FluidSimulation_disable_autosave
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_opencl_particle_advection(self):
        libfunc = lib.FluidSimulation_is_opencl_particle_advection_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_opencl_particle_advection.setter
    def enable_opencl_particle_advection(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_opencl_particle_advection
        else:
            libfunc = lib.FluidSimulation_disable_opencl_particle_advection
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def enable_opencl_scalar_field(self):
        libfunc = lib.FluidSimulation_is_opencl_scalar_field_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @enable_opencl_scalar_field.setter
    def enable_opencl_scalar_field(self, boolval):
        if boolval:
            libfunc = lib.FluidSimulation_enable_opencl_scalar_field
        else:
            libfunc = lib.FluidSimulation_disable_opencl_scalar_field
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def particle_advection_kernel_workload_size(self):
        libfunc = lib.FluidSimulation_get_particle_advection_kernel_workload_size
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @particle_advection_kernel_workload_size.setter
    @decorators.check_ge(1)
    def particle_advection_kernel_workload_size(self, size):
        libfunc = lib.FluidSimulation_set_particle_advection_kernel_workload_size
        pb.init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), size])

    @property
    def scalar_field_kernel_workload_size(self):
        libfunc = lib.FluidSimulation_get_scalar_field_kernel_workload_size
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @scalar_field_kernel_workload_size.setter
    @decorators.check_ge(1)
    def scalar_field_kernel_workload_size(self, size):
        libfunc = lib.FluidSimulation_set_scalar_field_kernel_workload_size
        pb.init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), size])

    @decorators.xyz_or_vector
    def add_body_force(self, fx, fy, fz):
        libfunc = lib.FluidSimulation_add_body_force
        pb.init_lib_func(
            libfunc, 
            [c_void_p, c_double, c_double, c_double, c_void_p], None
        )
        pb.execute_lib_func(libfunc, [self(), fx, fy, fz])

    def get_constant_body_force(self):
        libfunc = lib.FluidSimulation_get_constant_body_force
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], Vector3_t)
        cvect = pb.execute_lib_func(libfunc, [self()])
        return Vector3.from_struct(cvect)

    @decorators.xyz_or_vector
    def get_variable_body_force(self, px, py, pz):
        libfunc = lib.FluidSimulation_get_variable_body_force
        pb.init_lib_func(
            libfunc, 
            [c_void_p, c_double, c_double, c_double, c_void_p], Vector3_t
        )
        cvect = pb.execute_lib_func(libfunc, [self(), px, py, pz])
        return Vector3.from_struct(cvect)

    @decorators.xyz_or_vector
    def get_total_body_force(self, px, py, pz):
        libfunc = lib.FluidSimulation_get_total_body_force
        pb.init_lib_func(
            libfunc, 
            [c_void_p, c_double, c_double, c_double, c_void_p], Vector3_t
        )
        cvect = pb.execute_lib_func(libfunc, [self(), px, py, pz])
        return Vector3.from_struct(cvect)

    def reset_body_force(self):
        libfunc = lib.FluidSimulation_reset_body_force
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @_check_simulation_not_initialized
    @decorators.xyz_or_vector_and_radius
    def add_implicit_fluid_point(self, x, y, z, radius):
        libfunc = lib.FluidSimulation_add_implicit_fluid_point
        pb.init_lib_func(
            libfunc, 
            [c_void_p, c_double, c_double, c_double, c_double, c_void_p], None
        )
        pb.execute_lib_func(libfunc, [self(), x, y, z, radius])

    @_check_simulation_not_initialized
    @decorators.check_type(AABB)
    def add_fluid_cuboid(self, bbox):
        libfunc = lib.FluidSimulation_add_fluid_cuboid
        pb.init_lib_func(
            libfunc, [c_void_p, AABB_t, c_void_p], None
        )
        pb.execute_lib_func(libfunc, [self(), bbox.to_struct()])

    def add_spherical_fluid_source(self, source):
        libfunc = lib.FluidSimulation_add_spherical_fluid_source
        pb.init_lib_func(libfunc, [c_void_p, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), source()])
        pass

    def add_cuboid_fluid_source(self, source):
        libfunc = lib.FluidSimulation_add_cuboid_fluid_source
        pb.init_lib_func(libfunc, [c_void_p, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), source()])
        pass

    def remove_fluid_source(self, source):
        libfunc = lib.FluidSimulation_remove_fluid_source
        pb.init_lib_func(libfunc, [c_void_p, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), source()])
        pass

    def remove_fluid_sources(self):
        libfunc = lib.FluidSimulation_remove_fluid_sources
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    def add_solid_cells(self, cell_list):
        n = len(cell_list)
        indices = (GridIndex_t * n)()
        for i in range(n):
            indices[i].i = cell_list[i].i
            indices[i].j = cell_list[i].j
            indices[i].k = cell_list[i].k

        libfunc = lib.FluidSimulation_add_solid_cells
        pb.init_lib_func(libfunc, [c_void_p, c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), indices, n])

    def remove_solid_cells(self, cell_list):
        n = len(cell_list)
        indices = (GridIndex_t * n)()
        for i in range(n):
            indices[i].i = cell_list[i].i
            indices[i].j = cell_list[i].j
            indices[i].k = cell_list[i].k

        libfunc = lib.FluidSimulation_remove_solid_cells
        pb.init_lib_func(libfunc, [c_void_p, c_void_p, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), indices, n])

    def add_fluid_cells(self, cell_list, vx = 0.0, vy = 0.0, vz = 0.0):
        n = len(cell_list)
        indices = (GridIndex_t * n)()
        for i in range(n):
            indices[i].i = cell_list[i].i
            indices[i].j = cell_list[i].j
            indices[i].k = cell_list[i].k
        velocity = Vector3_t(vx, vy, vz)

        libfunc = lib.FluidSimulation_add_fluid_cells_velocity
        pb.init_lib_func(libfunc, [c_void_p, c_void_p, Vector3_t, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), indices, velocity, n])

    def remove_fluid_cells(self, cell_list):
        n = len(cell_list)
        indices = (GridIndex_t * n)()
        for i in range(n):
            indices[i].i = cell_list[i].i
            indices[i].j = cell_list[i].j
            indices[i].k = cell_list[i].k

        libfunc = lib.FluidSimulation_remove_fluid_cells
        pb.init_lib_func(libfunc, [c_void_p, c_void_p, c_int, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), indices, n])

    def get_num_marker_particles(self):
        libfunc = lib.FluidSimulation_get_num_marker_particles
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    def get_marker_particles(self, startidx = None, endidx = None):
        nparticles = self.get_num_marker_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (MarkerParticle_t * n)()

        libfunc = lib.FluidSimulation_get_marker_particles
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    def get_marker_particle_positions(self, startidx = None, endidx = None):
        nparticles = self.get_num_marker_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulation_get_marker_particle_positions
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    def get_marker_particle_velocities(self, startidx = None, endidx = None):
        nparticles = self.get_num_marker_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulation_get_marker_particle_velocities
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    def get_num_diffuse_particles(self):
        libfunc = lib.FluidSimulation_get_num_diffuse_particles
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    def get_diffuse_particles(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (DiffuseParticle_t * n)()

        libfunc = lib.FluidSimulation_get_diffuse_particles
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    def get_diffuse_particle_positions(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulation_get_diffuse_particle_positions
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    def get_diffuse_particle_velocities(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulation_get_diffuse_particle_velocities
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    def get_diffuse_particle_lifetimes(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (c_float * n)()

        libfunc = lib.FluidSimulation_get_diffuse_particle_lifetimes
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        floats = []
        for f in out:
            floats.append(float(f))

        return floats

    def get_diffuse_particle_types(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (c_char * n)()

        libfunc = lib.FluidSimulation_get_diffuse_particle_types
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        types = []
        for t in out:
            types.append(ord(t))

        return types

    def _check_range(self, startidx, endidx, minidx, maxidx):
        if startidx is None:
            startidx = minidx
        if endidx is None:
            endidx = maxidx

        if not isinstance(startidx, int) or not isinstance(endidx, int):
            raise TypeError("Index range must be integers")
        if startidx < minidx:
            raise IndexError("startidx out of range: " + str(startidx))
        if endidx > maxidx:
            raise IndexError("endidx out of range: " + str(endidx))
        if endidx < startidx:
            endidx = startidx

        return startidx, endidx

class MarkerParticle_t(ctypes.Structure):
    _fields_ = [("position", Vector3_t),
                ("velocity", Vector3_t)]

class DiffuseParticle_t(ctypes.Structure):
    _fields_ = [("position", Vector3_t),
                ("velocity", Vector3_t),
                ("lifetime", c_float),
                ("type", c_char)]