import ctypes
from ctypes import c_void_p
from ctypes import c_char_p
from ctypes import c_int
from ctypes import c_double
from ctypes import byref
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
        success = c_int()
        self._obj = lib.FluidSimulation_new_from_empty(byref(success))
        self._check_success(success, "FluidSimulation_new_from_empty - ")

    @decorators.check_gt_zero
    def _init_from_dimensions(self, isize, jsize, ksize, dx):
        success = c_int()
        self._obj = lib.FluidSimulation_new_from_dimensions(
                        isize, jsize, ksize, dx, byref(success))
        self._check_success(success, "FluidSimulation_new_from_dimensions - ")

    @classmethod
    @decorators.check_type(FluidSimulationSaveState)
    def from_save_state(cls, savestate):
        self = cls()
        lib.FluidSimulation_destroy(self._obj)

        success = c_int()
        self._obj = lib.FluidSimulation_new_from_save_state(savestate(), 
                                                            byref(success))
        self._check_success(success, "FluidSimulation_new_from_save_state - ")

        return self

    def __del__(self):
        try:
            lib.FluidSimulation_destroy(self._obj)
        except:
            pass

    def __call__(self):
        return self._obj

    def initialize(self):
        if self.is_initialized():
            return
        success = c_int()
        lib.FluidSimulation_initialize(self(), byref(success))
        self._check_success(success, "FluidSimulation_initialize - ")

    def is_initialized(self):
        success = c_int()
        result = lib.FluidSimulation_is_initialized(self(), byref(success))
        self._check_success(success, "FluidSimulation_is_initialized - ")
        return bool(result)

    @_check_simulation_initialized
    def update(self, dt):
        deltatime = c_double(dt)
        success = c_int()
        lib.FluidSimulation_update(self(), deltatime, byref(success))
        self._check_success(success, "FluidSimulation_update - ")

    @_check_simulation_initialized
    def save_state(self, filename):
        cfilename = c_char_p(filename)
        success = c_int()
        lib.FluidSimulation_save_state(self(), cfilename, byref(success))
        self._check_success(success, "FluidSimulation_save_state - ")

    def get_current_frame(self):
        success = c_int()
        frameno = lib.FluidSimulation_get_current_frame(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_current_frame - ")
        return int(frameno)

    def is_current_frame_finished(self):
        success = c_int()
        result = lib.FluidSimulation_is_current_frame_finished(self(), byref(success))
        self._check_success(success, "FluidSimulation_is_current_frame_finished - ")
        return bool(result)

    def get_cell_size(self):
        success = c_int()
        dx = lib.FluidSimulation_get_cell_size(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_cell_size - ")
        return dx

    def get_grid_dimensions(self):
        isize = c_int()
        jsize = c_int()
        ksize = c_int()
        success = c_int()
        lib.FluidSimulation_get_grid_dimensions(self(), byref(isize),
                                                        byref(jsize),
                                                        byref(ksize),
                                                        byref(success))
        self._check_success(success, "FluidSimulation_get_grid_dimensions - ")
        return GridIndex(isize.value, jsize.value, ksize.value)

    def get_grid_width(self):
        success = c_int()
        isize = lib.FluidSimulation_get_grid_width(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_grid_width - ")
        return isize

    def get_grid_height(self):
        success = c_int()
        jsize = lib.FluidSimulation_get_grid_height(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_grid_height - ")
        return jsize

    def get_grid_depth(self):
        success = c_int()
        ksize = lib.FluidSimulation_get_grid_depth(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_grid_depth - ")
        return ksize

    def get_simulation_dimensions(self):
        width = c_double()
        height = c_double()
        depth = c_double()
        success = c_int()
        lib.FluidSimulation_get_simulation_dimensions(self(), byref(width),
                                                              byref(height),
                                                              byref(depth),
                                                              byref(success))
        self._check_success(success, "FluidSimulation_get_simulation_dimensions - ")
        return Vector3(width.value, height.value, depth.value)

    def get_simulation_width(self):
        success = c_int()
        width = lib.FluidSimulation_get_simulation_width(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_simulation_width - ")
        return width

    def get_simulation_height(self):
        success = c_int()
        height = lib.FluidSimulation_get_simulation_height(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_simulation_height - ")
        return height

    def get_simulation_depth(self):
        success = c_int()
        depth = lib.FluidSimulation_get_simulation_depth(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_simulation_depth - ")
        return depth

    @property
    def density(self):
        success = c_int()
        density = lib.FluidSimulation_get_density(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_density - ")
        return density

    @density.setter
    @decorators.check_gt_zero
    def density(self, value):
        cval = c_double(value)
        success = c_int()
        lib.FluidSimulation_set_density(self(), cval, byref(success))
        self._check_success(success, "FluidSimulation_set_density - ")

    @decorators.ijk_or_gridindex
    def get_material(self, i, j, k):
        ci = c_int(i)
        cj = c_int(j)
        ck = c_int(k)
        success = c_int()
        m = lib.FluidSimulation_get_material(self(), ci, cj, ck, byref(success))
        self._check_success(success, "FluidSimulation_add_implicit_fluid_point - ")
        return m

    @property
    def marker_particle_scale(self):
        success = c_int()
        scale = lib.FluidSimulation_get_marker_particle_scale(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_marker_particle_scale - ")
        return scale

    @marker_particle_scale.setter
    @decorators.check_ge_zero
    def marker_particle_scale(self, scale):
        cval = c_double(scale)
        success = c_int()
        lib.FluidSimulation_set_marker_particle_scale(self(), cval, byref(success))
        self._check_success(success, "FluidSimulation_set_marker_particle_scale - ")

    @property
    def surface_subdivision_level(self):
        success = c_int()
        level = lib.FluidSimulation_get_surface_subdivision_level(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_surface_subdivision_level - ")
        return level

    @surface_subdivision_level.setter
    @decorators.check_ge(1)
    def surface_subdivision_level(self, level):
        cval = c_int(level)
        success = c_int()
        lib.FluidSimulation_set_surface_subdivision_level(self(), cval, byref(success))
        self._check_success(success, "FluidSimulation_set_surface_subdivision_level - ")

    @property
    def num_polygonizer_slices(self):
        success = c_int()
        slices = lib.FluidSimulation_get_num_polygonizer_slices(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_num_polygonizer_slices - ")
        return slices

    @num_polygonizer_slices.setter
    @decorators.check_ge(1)
    def num_polygonizer_slices(self, slices):
        cval = c_int(slices)
        success = c_int()
        lib.FluidSimulation_set_num_polygonizer_slices(self(), cval, byref(success))
        self._check_success(success, "FluidSimulation_set_num_polygonizer_slices - ")

    @property
    def min_polyhedron_triangle_count(self):
        success = c_int()
        count = lib.FluidSimulation_get_min_polyhedron_triangle_count(self(), 
                                                                      byref(success))
        self._check_success(success, "FluidSimulation_get_min_polyhedron_triangle_count - ")
        return count

    @min_polyhedron_triangle_count.setter
    @decorators.check_ge_zero
    def min_polyhedron_triangle_count(self, count):
        cval = c_int(count)
        success = c_int()
        lib.FluidSimulation_set_min_polyhedron_triangle_count(self(), cval, 
                                                              byref(success))
        self._check_success(success, "FluidSimulation_set_min_polyhedron_triangle_count - ")

    @property
    def enable_surface_mesh_output(self):
        success = c_int()
        result = lib.FluidSimulation_is_surface_mesh_output_enabled(self(), 
                                                                    byref(success))
        self._check_success(success, "FluidSimulation_is_surface_mesh_output_enabled - ")
        return bool(result)

    @enable_surface_mesh_output.setter
    def enable_surface_mesh_output(self, boolval):
        success = c_int()
        if boolval:
            lib.FluidSimulation_enable_surface_mesh_output(self(), byref(success))
            self._check_success(success, "FluidSimulation_enable_surface_mesh_output - ")
        else:
            lib.FluidSimulation_disable_surface_mesh_output(self(), byref(success))
            self._check_success(success, "FluidSimulation_disable_surface_mesh_output - ")

    @property
    def enable_isotropic_surface_reconstruction(self):
        success = c_int()
        result = lib.FluidSimulation_is_isotropic_surface_reconstruction_enabled(self(), 
                                                                              byref(success))
        self._check_success(success, "FluidSimulation_is_isotropic_surface_reconstruction_enabled - ")
        return bool(result)

    @enable_isotropic_surface_reconstruction.setter
    def enable_isotropic_surface_reconstruction(self, boolval):
        success = c_int()
        if boolval:
            lib.FluidSimulation_enable_isotropic_surface_reconstruction(self(), byref(success))
            self._check_success(success, "FluidSimulation_enable_isotropic_surface_reconstruction - ")
        else:
            lib.FluidSimulation_disable_isotropic_surface_reconstruction(self(), byref(success))
            self._check_success(success, "FluidSimulation_disable_isotropic_surface_reconstruction - ")

    @property
    def enable_anisotropic_surface_reconstruction(self):
        success = c_int()
        result = lib.FluidSimulation_is_anisotropic_surface_reconstruction_enabled(self(), 
                                                                                   byref(success))
        self._check_success(success, "FluidSimulation_is_anisotropic_surface_reconstruction_enabled - ")
        return bool(result)

    @enable_anisotropic_surface_reconstruction.setter
    def enable_anisotropic_surface_reconstruction(self, boolval):
        success = c_int()
        if boolval:
            lib.FluidSimulation_enable_anisotropic_surface_reconstruction(self(), byref(success))
            self._check_success(success, "FluidSimulation_enable_anisotropic_surface_reconstruction - ")
        else:
            lib.FluidSimulation_disable_anisotropic_surface_reconstruction(self(), byref(success))
            self._check_success(success, "FluidSimulation_disable_anisotropic_surface_reconstruction - ")

    @property
    def enable_diffuse_material_output(self):
        success = c_int()
        result = lib.FluidSimulation_is_diffuse_material_output_enabled(self(), 
                                                                        byref(success))
        self._check_success(success, "FluidSimulation_is_diffuse_material_output_enabled - ")
        return bool(result)

    @enable_diffuse_material_output.setter
    def enable_diffuse_material_output(self, boolval):
        success = c_int()
        if boolval:
            lib.FluidSimulation_enable_diffuse_material_output(self(), byref(success))
            self._check_success(success, "FluidSimulation_enable_diffuse_material_output - ")
        else:
            lib.FluidSimulation_disable_diffuse_material_output(self(), byref(success))
            self._check_success(success, "FluidSimulation_disable_diffuse_material_output - ")

    @property
    def enable_bubble_diffuse_material(self):
        success = c_int()
        result = lib.FluidSimulation_is_bubble_diffuse_material_enabled(self(), 
                                                                        byref(success))
        self._check_success(success, "FluidSimulation_is_bubble_diffuse_material_enabled - ")
        return bool(result)

    @enable_bubble_diffuse_material.setter
    def enable_bubble_diffuse_material(self, boolval):
        success = c_int()
        if boolval:
            lib.FluidSimulation_enable_bubble_diffuse_material(self(), byref(success))
            self._check_success(success, "FluidSimulation_enable_bubble_diffuse_material - ")
        else:
            lib.FluidSimulation_disable_bubble_diffuse_material(self(), byref(success))
            self._check_success(success, "FluidSimulation_disable_bubble_diffuse_material - ")

    @property
    def enable_spray_diffuse_material(self):
        success = c_int()
        result = lib.FluidSimulation_is_spray_diffuse_material_enabled(self(), 
                                                                       byref(success))
        self._check_success(success, "FluidSimulation_is_spray_diffuse_material_enabled - ")
        return bool(result)

    @enable_spray_diffuse_material.setter
    def enable_spray_diffuse_material(self, boolval):
        success = c_int()
        if boolval:
            lib.FluidSimulation_enable_spray_diffuse_material(self(), byref(success))
            self._check_success(success, "FluidSimulation_enable_spray_diffuse_material - ")
        else:
            lib.FluidSimulation_disable_spray_diffuse_material(self(), byref(success))
            self._check_success(success, "FluidSimulation_disable_spray_diffuse_material - ")

    @property
    def enable_foam_diffuse_material(self):
        success = c_int()
        result = lib.FluidSimulation_is_foam_diffuse_material_enabled(self(), 
                                                                      byref(success))
        self._check_success(success, "FluidSimulation_is_foam_diffuse_material_enabled - ")
        return bool(result)

    @enable_foam_diffuse_material.setter
    def enable_foam_diffuse_material(self, boolval):
        success = c_int()
        if boolval:
            lib.FluidSimulation_enable_foam_diffuse_material(self(), byref(success))
            self._check_success(success, "FluidSimulation_enable_foam_diffuse_material - ")
        else:
            lib.FluidSimulation_disable_foam_diffuse_material(self(), byref(success))
            self._check_success(success, "FluidSimulation_disable_foam_diffuse_material - ")

    @property
    def max_num_diffuse_particles(self):
        success = c_int()
        n = lib.FluidSimulation_get_max_num_diffuse_particles(self(), byref(success))
        self._check_success(success, "FluidSimulation_get_max_num_diffuse_particles - ")
        return n

    @max_num_diffuse_particles.setter
    @decorators.check_ge_zero
    def max_num_diffuse_particles(self, num):
        cval = c_int(num)
        success = c_int()
        lib.FluidSimulation_set_max_num_diffuse_particles(self(), cval, byref(success))
        self._check_success(success, "FluidSimulation_set_max_num_diffuse_particles - ")

    @property
    def diffuse_particle_wavecrest_emission_rate(self):
        success = c_int()
        rate = lib.FluidSimulation_get_diffuse_particle_wavecrest_emission_rate(self(), 
                                                                                byref(success))
        self._check_success(success, "FluidSimulation_get_diffuse_particle_wavecrest_emission_rate - ")
        return rate

    @diffuse_particle_wavecrest_emission_rate.setter
    @decorators.check_ge_zero
    def diffuse_particle_wavecrest_emission_rate(self, rate):
        cval = c_double(rate)
        success = c_int()
        lib.FluidSimulation_set_diffuse_particle_wavecrest_emission_rate(self(), 
                                                                         cval, 
                                                                         byref(success))
        self._check_success(success, "FluidSimulation_set_diffuse_particle_wavecrest_emission_rate - ")

    @property
    def diffuse_particle_turbulence_emission_rate(self):
        success = c_int()
        rate = lib.FluidSimulation_get_diffuse_particle_turbulence_emission_rate(self(), 
                                                                                 byref(success))
        self._check_success(success, "FluidSimulation_get_diffuse_particle_turbulence_emission_rate - ")
        return rate

    @diffuse_particle_turbulence_emission_rate.setter
    @decorators.check_ge_zero
    def diffuse_particle_turbulence_emission_rate(self, rate):
        cval = c_double(rate)
        success = c_int()
        lib.FluidSimulation_set_diffuse_particle_turbulence_emission_rate(self(), 
                                                                          cval, 
                                                                          byref(success))
        self._check_success(success, "FluidSimulation_set_diffuse_particle_turbulence_emission_rate - ")

    @_check_simulation_not_initialized
    @decorators.xyz_or_vector_and_radius
    def add_implicit_fluid_point(self, x, y, z, radius):
        x = c_double(x)
        y = c_double(y)
        z = c_double(z)
        radius = c_double(radius)
        success = c_int()
        lib.FluidSimulation_add_implicit_fluid_point(self(), x, y, z, radius, 
                                                     byref(success))
        self._check_success(success, "FluidSimulation_add_implicit_fluid_point - ")

    def _check_success(self, success, errprefix):
        if not success:
            errmsg = errprefix + lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)


def __init__(lib):
    lib.FluidSimulation_new_from_empty.argtypes = [c_void_p]
    lib.FluidSimulation_new_from_empty.restype = c_void_p

    lib.FluidSimulation_new_from_dimensions.argtypes = [c_int, c_int, c_int,
                                                        c_double, c_void_p]
    lib.FluidSimulation_new_from_dimensions.restype = c_void_p

    lib.FluidSimulation_new_from_save_state.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_new_from_save_state.restype = c_void_p

    lib.FluidSimulation_destroy.argtypes = [c_void_p]
    lib.FluidSimulation_destroy.restype = None

    lib.FluidSimulation_initialize.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_initialize.restype = None

    lib.FluidSimulation_is_initialized.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_initialized.restype = c_int

    lib.FluidSimulation_update.argtypes = [c_void_p, c_double, c_void_p]
    lib.FluidSimulation_update.restype = None

    lib.FluidSimulation_save_state.argtypes = [c_void_p, c_char_p, c_void_p]
    lib.FluidSimulation_save_state.restype = None

    lib.FluidSimulation_get_current_frame.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_current_frame.restype = c_int

    lib.FluidSimulation_is_current_frame_finished.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_current_frame_finished.restype = c_int

    lib.FluidSimulation_get_cell_size.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_cell_size.restype = c_double

    lib.FluidSimulation_get_grid_dimensions.argtypes = [c_void_p, c_void_p, c_void_p,
                                                        c_void_p, c_void_p]
    lib.FluidSimulation_get_grid_dimensions.restype = None

    lib.FluidSimulation_get_grid_width.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_grid_width.restype = c_int

    lib.FluidSimulation_get_grid_height.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_grid_height.restype = c_int

    lib.FluidSimulation_get_grid_depth.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_grid_depth.restype = c_int

    lib.FluidSimulation_get_simulation_dimensions.argtypes = [c_void_p, c_void_p,
                                                              c_void_p, c_void_p,
                                                              c_void_p]
    lib.FluidSimulation_get_simulation_dimensions.restype = None

    lib.FluidSimulation_get_simulation_width.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_simulation_width.restype = c_double

    lib.FluidSimulation_get_simulation_height.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_simulation_height.restype = c_double

    lib.FluidSimulation_get_simulation_depth.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_simulation_depth.restype = c_double

    lib.FluidSimulation_get_density.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_density.restype = c_double

    lib.FluidSimulation_set_density.argtypes = [c_void_p, c_double, c_void_p]
    lib.FluidSimulation_set_density.restype = None

    lib.FluidSimulation_get_material.argtypes = [c_void_p, c_int, c_int, c_int,
                                                 c_void_p]
    lib.FluidSimulation_get_material.restype = c_int

    lib.FluidSimulation_get_marker_particle_scale.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_marker_particle_scale.restype = c_double

    lib.FluidSimulation_set_marker_particle_scale.argtypes = [c_void_p, c_double, c_void_p]
    lib.FluidSimulation_set_marker_particle_scale.restype = None

    lib.FluidSimulation_get_surface_subdivision_level.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_surface_subdivision_level.restype = c_int

    lib.FluidSimulation_set_surface_subdivision_level.argtypes = [c_void_p, c_int, c_void_p]
    lib.FluidSimulation_set_surface_subdivision_level.restype = None

    lib.FluidSimulation_get_num_polygonizer_slices.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_num_polygonizer_slices.restype = c_int

    lib.FluidSimulation_set_num_polygonizer_slices.argtypes = [c_void_p, c_int, c_void_p]
    lib.FluidSimulation_set_num_polygonizer_slices.restype = None

    lib.FluidSimulation_get_min_polyhedron_triangle_count.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_min_polyhedron_triangle_count.restype = c_int

    lib.FluidSimulation_set_min_polyhedron_triangle_count.argtypes = [c_void_p, c_int, c_void_p]
    lib.FluidSimulation_set_min_polyhedron_triangle_count.restype = None

    lib.FluidSimulation_enable_surface_mesh_output.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_enable_surface_mesh_output.restype = None

    lib.FluidSimulation_disable_surface_mesh_output.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_disable_surface_mesh_output.restype = None

    lib.FluidSimulation_is_surface_mesh_output_enabled.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_surface_mesh_output_enabled.restype = c_int

    lib.FluidSimulation_enable_isotropic_surface_reconstruction.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_enable_isotropic_surface_reconstruction.restype = None

    lib.FluidSimulation_disable_isotropic_surface_reconstruction.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_disable_isotropic_surface_reconstruction.restype = None

    lib.FluidSimulation_is_isotropic_surface_reconstruction_enabled.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_isotropic_surface_reconstruction_enabled.restype = c_int

    lib.FluidSimulation_enable_anisotropic_surface_reconstruction.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_enable_anisotropic_surface_reconstruction.restype = None

    lib.FluidSimulation_disable_anisotropic_surface_reconstruction.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_disable_anisotropic_surface_reconstruction.restype = None

    lib.FluidSimulation_is_anisotropic_surface_reconstruction_enabled.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_anisotropic_surface_reconstruction_enabled.restype = c_int

    lib.FluidSimulation_enable_diffuse_material_output.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_enable_diffuse_material_output.restype = None

    lib.FluidSimulation_disable_diffuse_material_output.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_disable_diffuse_material_output.restype = None

    lib.FluidSimulation_is_diffuse_material_output_enabled.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_diffuse_material_output_enabled.restype = c_int

    lib.FluidSimulation_enable_bubble_diffuse_material.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_enable_bubble_diffuse_material.restype = None

    lib.FluidSimulation_disable_bubble_diffuse_material.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_disable_bubble_diffuse_material.restype = None

    lib.FluidSimulation_is_bubble_diffuse_material_enabled.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_bubble_diffuse_material_enabled.restype = c_int

    lib.FluidSimulation_enable_spray_diffuse_material.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_enable_spray_diffuse_material.restype = None

    lib.FluidSimulation_disable_spray_diffuse_material.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_disable_spray_diffuse_material.restype = None

    lib.FluidSimulation_is_spray_diffuse_material_enabled.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_spray_diffuse_material_enabled.restype = c_int

    lib.FluidSimulation_enable_foam_diffuse_material.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_enable_foam_diffuse_material.restype = None

    lib.FluidSimulation_disable_foam_diffuse_material.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_disable_foam_diffuse_material.restype = None

    lib.FluidSimulation_is_foam_diffuse_material_enabled.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_is_foam_diffuse_material_enabled.restype = c_int

    lib.FluidSimulation_get_max_num_diffuse_particles.argtypes = [c_void_p, c_void_p]
    lib.FluidSimulation_get_max_num_diffuse_particles.restype = c_int

    lib.FluidSimulation_set_max_num_diffuse_particles.argtypes = [c_void_p, c_int, c_void_p]
    lib.FluidSimulation_set_max_num_diffuse_particles.restype = None

    lib.FluidSimulation_get_diffuse_particle_wavecrest_emission_rate.argtypes = [c_void_p, 
                                                                                 c_void_p]
    lib.FluidSimulation_get_diffuse_particle_wavecrest_emission_rate.restype = c_double

    lib.FluidSimulation_set_diffuse_particle_wavecrest_emission_rate.argtypes = [c_void_p, 
                                                                                 c_double, 
                                                                                 c_void_p]
    lib.FluidSimulation_set_diffuse_particle_wavecrest_emission_rate.restype = None

    lib.FluidSimulation_get_diffuse_particle_turbulence_emission_rate.argtypes = [c_void_p, 
                                                                                 c_void_p]
    lib.FluidSimulation_get_diffuse_particle_turbulence_emission_rate.restype = c_double

    lib.FluidSimulation_set_diffuse_particle_turbulence_emission_rate.argtypes = [c_void_p, 
                                                                                 c_double, 
                                                                                 c_void_p]
    lib.FluidSimulation_set_diffuse_particle_turbulence_emission_rate.restype = None

    lib.FluidSimulation_add_implicit_fluid_point.argtypes = [c_void_p, c_double,
                                                             c_double, c_double,
                                                             c_double, c_void_p]
    lib.FluidSimulation_add_implicit_fluid_point.restype = None

    lib.FluidSimulation_get_error_message.argtypes = []
    lib.FluidSimulation_get_error_message.restype = c_char_p

    

__init__(lib)