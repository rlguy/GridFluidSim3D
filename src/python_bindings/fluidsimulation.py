from ctypes import c_void_p, c_char_p, c_int, c_double, byref
import numbers

from fluidlib import lib
from fluidsimulationsavestate import FluidSimulationSaveState
from vector3 import Vector3
from gridindex import GridIndex
import method_decorators as decorators


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
        self._init_lib_func(libfunc, [c_void_p], c_void_p)
        self._obj = self._execute_lib_func(libfunc, [])

    @decorators.check_gt_zero
    def _init_from_dimensions(self, isize, jsize, ksize, dx):
        libfunc = lib.FluidSimulation_new_from_dimensions
        self._init_lib_func(libfunc, 
                            [c_int, c_int, c_int, c_double, c_void_p], 
                            c_void_p)
        self._obj = self._execute_lib_func(libfunc, [isize, jsize, ksize, dx])

    @classmethod
    @decorators.check_type(FluidSimulationSaveState)
    def from_save_state(cls, savestate):
        self = cls()
        lib.FluidSimulation_destroy(self._obj)

        libfunc = lib.FluidSimulation_new_from_save_state
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_void_p)
        self._obj = self._execute_lib_func(libfunc, [savestate()])
        return self

    def __del__(self):
        libfunc = lib.FluidSimulation_destroy
        self._init_lib_func(libfunc, [c_void_p], None)
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
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    def is_initialized(self):
        libfunc = lib.FluidSimulation_is_initialized
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @_check_simulation_initialized
    def update(self, dt):
        libfunc = lib.FluidSimulation_update
        self._init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), dt])

    @_check_simulation_initialized
    def save_state(self, filename):
        libfunc = lib.FluidSimulation_save_state
        self._init_lib_func(libfunc, [c_void_p, c_char_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), filename])

    def get_current_frame(self):
        libfunc = lib.FluidSimulation_get_current_frame
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    def is_current_frame_finished(self):
        libfunc = lib.FluidSimulation_is_current_frame_finished
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    def get_cell_size(self):
        libfunc = lib.FluidSimulation_get_cell_size
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    def get_grid_dimensions(self):
        libfunc = lib.FluidSimulation_get_grid_dimensions
        if libfunc.argtypes is None:
            libfunc.argtypes = [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p]
            libfunc.restype = None
        self._init_lib_func(libfunc, 
                            [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p], 
                            None)
        isize = c_int()
        jsize = c_int()
        ksize = c_int()
        success = c_int()
        libfunc(self(), byref(isize), byref(jsize), byref(ksize), byref(success))
        self._check_success(success, libfunc.__name__ + " - ")

        return GridIndex(isize.value, jsize.value, ksize.value)

    def get_grid_width(self):
        libfunc = lib.FluidSimulation_get_grid_width
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    def get_grid_height(self):
        libfunc = lib.FluidSimulation_get_grid_height
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    def get_grid_depth(self):
        libfunc = lib.FluidSimulation_get_grid_depth
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    def get_simulation_dimensions(self):
        libfunc = lib.FluidSimulation_get_simulation_dimensions
        self._init_lib_func(libfunc, 
                            [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p], 
                            None)
        width = c_double()
        height = c_double()
        depth = c_double()
        success = c_int()
        libfunc(self(), byref(width), byref(height), byref(depth), byref(success))
        self._check_success(success, libfunc.__name__ + " - ")

        return Vector3(width.value, height.value, depth.value)

    def get_simulation_width(self):
        libfunc = lib.FluidSimulation_get_simulation_width
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    def get_simulation_height(self):
        libfunc = lib.FluidSimulation_get_simulation_height
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    def get_simulation_depth(self):
        libfunc = lib.FluidSimulation_get_simulation_depth
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    @property
    def density(self):
        libfunc = lib.FluidSimulation_get_density
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    @density.setter
    @decorators.check_gt_zero
    def density(self, value):
        libfunc = lib.FluidSimulation_set_density
        self._init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), value])


    @decorators.ijk_or_gridindex
    def get_material(self, i, j, k):
        libfunc = lib.FluidSimulation_get_material
        self._init_lib_func(libfunc, [c_void_p, c_int, c_int, c_int, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self(), i, j, k])

    @property
    def marker_particle_scale(self):
        libfunc = lib.FluidSimulation_get_marker_particle_scale
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    @marker_particle_scale.setter
    @decorators.check_ge_zero
    def marker_particle_scale(self, scale):
        libfunc = lib.FluidSimulation_set_marker_particle_scale
        self._init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), scale])

    @property
    def surface_subdivision_level(self):
        libfunc = lib.FluidSimulation_get_surface_subdivision_level
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    @surface_subdivision_level.setter
    @decorators.check_ge(1)
    def surface_subdivision_level(self, level):
        libfunc = lib.FluidSimulation_set_surface_subdivision_level
        self._init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), level])

    @property
    def num_polygonizer_slices(self):
        libfunc = lib.FluidSimulation_get_num_polygonizer_slices
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    @num_polygonizer_slices.setter
    @decorators.check_ge(1)
    def num_polygonizer_slices(self, slices):
        libfunc = lib.FluidSimulation_set_num_polygonizer_slices
        self._init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), slices])

    @property
    def min_polyhedron_triangle_count(self):
        libfunc = lib.FluidSimulation_get_min_polyhedron_triangle_count
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    @min_polyhedron_triangle_count.setter
    @decorators.check_ge_zero
    def min_polyhedron_triangle_count(self, count):
        libfunc = lib.FluidSimulation_set_min_polyhedron_triangle_count
        self._init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), count])

    @property
    def enable_surface_mesh_output(self):
        libfunc = lib.FluidSimulation_is_surface_mesh_output_enabled
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @enable_surface_mesh_output.setter
    def enable_surface_mesh_output(self, boolval):
        libfunc = None
        if boolval:
            libfunc = lib.FluidSimulation_enable_surface_mesh_output
        else:
            libfunc = lib.FluidSimulation_disable_surface_mesh_output
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    @property
    def enable_isotropic_surface_reconstruction(self):
        libfunc = lib.FluidSimulation_is_isotropic_surface_reconstruction_enabled
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @enable_isotropic_surface_reconstruction.setter
    def enable_isotropic_surface_reconstruction(self, boolval):
        libfunc = None
        if boolval:
            libfunc = lib.FluidSimulation_enable_isotropic_surface_reconstruction
        else:
            libfunc = lib.FluidSimulation_disable_isotropic_surface_reconstruction
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    @property
    def enable_anisotropic_surface_reconstruction(self):
        libfunc = lib.FluidSimulation_is_anisotropic_surface_reconstruction_enabled
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @enable_anisotropic_surface_reconstruction.setter
    def enable_anisotropic_surface_reconstruction(self, boolval):
        libfunc = None
        if boolval:
            libfunc = lib.FluidSimulation_enable_anisotropic_surface_reconstruction
        else:
            libfunc = lib.FluidSimulation_disable_anisotropic_surface_reconstruction
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    @property
    def enable_diffuse_material_output(self):
        libfunc = lib.FluidSimulation_is_diffuse_material_output_enabled
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @enable_diffuse_material_output.setter
    def enable_diffuse_material_output(self, boolval):
        libfunc = None
        if boolval:
            libfunc = lib.FluidSimulation_enable_diffuse_material_output
        else:
            libfunc = lib.FluidSimulation_disable_diffuse_material_output
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    @property
    def enable_bubble_diffuse_material(self):
        libfunc = lib.FluidSimulation_is_bubble_diffuse_material_enabled
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @enable_bubble_diffuse_material.setter
    def enable_bubble_diffuse_material(self, boolval):
        libfunc = None
        if boolval:
            libfunc = lib.FluidSimulation_enable_bubble_diffuse_material
        else:
            libfunc = lib.FluidSimulation_disable_bubble_diffuse_material
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    @property
    def enable_spray_diffuse_material(self):
        libfunc = lib.FluidSimulation_is_spray_diffuse_material_enabled
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @enable_spray_diffuse_material.setter
    def enable_spray_diffuse_material(self, boolval):
        libfunc = None
        if boolval:
            libfunc = lib.FluidSimulation_enable_spray_diffuse_material
        else:
            libfunc = lib.FluidSimulation_disable_spray_diffuse_material
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    @property
    def enable_foam_diffuse_material(self):
        libfunc = lib.FluidSimulation_is_foam_diffuse_material_enabled
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(self._execute_lib_func(libfunc, [self()]))

    @enable_foam_diffuse_material.setter
    def enable_foam_diffuse_material(self, boolval):
        libfunc = None
        if boolval:
            libfunc = lib.FluidSimulation_get_max_num_diffuse_particles
        else:
            libfunc = lib.FluidSimulation_disable_foam_diffuse_material
        self._init_lib_func(libfunc, [c_void_p, c_void_p], None)
        self._execute_lib_func(libfunc, [self()])

    @property
    def max_num_diffuse_particles(self):
        libfunc = lib.FluidSimulation_get_max_num_diffuse_particles
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return self._execute_lib_func(libfunc, [self()])

    @max_num_diffuse_particles.setter
    def max_num_diffuse_particles(self, num):
        libfunc = lib.FluidSimulation_set_max_num_diffuse_particles
        self._init_lib_func(libfunc, [c_void_p, c_int, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), num])

    @property
    def diffuse_particle_wavecrest_emission_rate(self):
        libfunc = lib.FluidSimulation_get_diffuse_particle_wavecrest_emission_rate
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    @diffuse_particle_wavecrest_emission_rate.setter
    @decorators.check_ge_zero
    def diffuse_particle_wavecrest_emission_rate(self, rate):
        libfunc = lib.FluidSimulation_set_diffuse_particle_wavecrest_emission_rate
        self._init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), rate])

    @property
    def diffuse_particle_turbulence_emission_rate(self):
        libfunc = lib.FluidSimulation_get_diffuse_particle_turbulence_emission_rate
        self._init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return self._execute_lib_func(libfunc, [self()])

    @diffuse_particle_turbulence_emission_rate.setter
    @decorators.check_ge_zero
    def diffuse_particle_turbulence_emission_rate(self, rate):
        libfunc = lib.FluidSimulation_set_diffuse_particle_turbulence_emission_rate
        self._init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        self._execute_lib_func(libfunc, [self(), rate])

    @_check_simulation_not_initialized
    @decorators.xyz_or_vector_and_radius
    def add_implicit_fluid_point(self, x, y, z, radius):
        libfunc = lib.FluidSimulation_add_implicit_fluid_point
        self._init_lib_func(
            libfunc, 
            [c_void_p, c_double, c_double, c_double, c_double, c_void_p], None
        )
        self._execute_lib_func(libfunc, [self(), x, y, z, radius])

    def _check_success(self, success, errprefix):
        libfunc = lib.FluidSimulation_get_error_message
        self._init_lib_func(libfunc, [], c_char_p)
        if not success:
            errmsg = errprefix + lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)

    def _init_lib_func(self, libfunc, argtypes, restype):
        if libfunc.argtypes is None:
            libfunc.argtypes = argtypes
            libfunc.restype = restype

    def _execute_lib_func(self, libfunc, params):
        args = []
        for idx, arg in enumerate(params):
            cval = libfunc.argtypes[idx](arg)
            args.append(cval)
        success = c_int();
        args.append(byref(success))

        result = None
        if libfunc.restype:
            result = libfunc.restype(libfunc(*args)).value
        else:
            libfunc(*args)

        self._check_success(success, libfunc.__name__ + " - ")
        return result
