import ctypes
from ctypes import c_void_p, c_char_p, c_int, c_double, byref

from .pyfluid import pyfluid as lib
from .vector3 import Vector3, Vector3_t
from .gridindex import GridIndex, GridIndex_t
from . import pybindings as pb

def _check_load_state_initialized(func):
    def wrapper(*args, **kwargs):
        self = args[0]
        if isinstance(self, FluidSimulationSaveState):
            self._check_load_state()
        return func(*args, **kwargs)

    return wrapper

class FluidSimulationSaveState(object):
    def __init__(self):
        libfunc = lib.FluidSimulationSaveState_new
        pb.init_lib_func(libfunc, [c_void_p], c_void_p)
        self._obj = pb.execute_lib_func(libfunc, [])

    def __del__(self):
        libfunc = lib.FluidSimulationSaveState_destroy
        pb.init_lib_func(libfunc, [c_void_p], None)
        try:
            libfunc(self._obj)
        except:
            pass

    def __call__(self):
        return self._obj

    def save_state(self, filename, fluidsimulation):
        libfunc = lib.FluidSimulationSaveState_save_state
        pb.init_lib_func(libfunc, [c_void_p, c_char_p, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), filename, fluidsimulation()])

    def load_state(self, filename):
        libfunc = lib.FluidSimulationSaveState_load_state
        pb.init_lib_func(libfunc, [c_void_p, c_char_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self(), filename]))

    def close_state(self):
        libfunc = lib.FluidSimulationSaveState_close_state
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @_check_load_state_initialized
    def get_grid_dimensions(self):
        i = ctypes.c_int()
        j = ctypes.c_int()
        k = ctypes.c_int()
        success = ctypes.c_int()
        libfunc = lib.FluidSimulationSaveState_get_grid_dimensions
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p], None)
        libfunc(self(), byref(i), byref(j), byref(k), byref(success))
        pb.check_success(success, libfunc.__name__ + " - ")

        return GridIndex(i.value, j.value, k.value)

    @_check_load_state_initialized
    def get_cell_size(self):
        libfunc = lib.FluidSimulationSaveState_get_cell_size
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @_check_load_state_initialized
    def get_current_frame(self):
        libfunc = lib.FluidSimulationSaveState_get_current_frame
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @_check_load_state_initialized
    def get_num_marker_particles(self):
        libfunc = lib.FluidSimulationSaveState_get_num_marker_particles
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])
        
    @_check_load_state_initialized
    def get_num_diffuse_particles(self):
        libfunc = lib.FluidSimulationSaveState_get_num_diffuse_particles
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])
        
    @_check_load_state_initialized
    def get_num_solid_cells(self):
        libfunc = lib.FluidSimulationSaveState_get_num_solid_cells
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    @_check_load_state_initialized
    def get_marker_particle_positions(self, startidx = None, endidx = None):
        nparticles = self.get_num_marker_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulationSaveState_get_marker_particle_positions
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    @_check_load_state_initialized
    def get_marker_particle_velocities(self, startidx = None, endidx = None):
        nparticles = self.get_num_marker_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulationSaveState_get_marker_particle_velocities
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    @_check_load_state_initialized
    def get_diffuse_particle_positions(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0,  nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulationSaveState_get_diffuse_particle_positions
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    @_check_load_state_initialized
    def get_diffuse_particle_velocities(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0,  nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()

        libfunc = lib.FluidSimulationSaveState_get_diffuse_particle_velocities
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    @_check_load_state_initialized
    def get_diffuse_particle_lifetimes(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0,  nparticles)
        n = endidx - startidx
        out = (ctypes.c_float * n)()

        libfunc = lib.FluidSimulationSaveState_get_diffuse_particle_lifetimes
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        lifetimes = [0.0]*n
        for i in range(n):
            lifetimes[i] = out[i]
        return lifetimes

    @_check_load_state_initialized
    def get_diffuse_particle_types(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (ctypes.c_char * n)()

        libfunc = lib.FluidSimulationSaveState_get_diffuse_particle_types
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        types = [0]*n
        for i in range(n):
            types[i] = ord(out[i])
        return types

    @_check_load_state_initialized
    def get_solid_cells(self, startidx = None, endidx = None):
        ncells = self.get_num_solid_cells()
        startidx, endidx = self._check_range(startidx, endidx, 0, ncells)
        n = endidx - startidx
        out = (GridIndex_t * n)()

        libfunc = lib.FluidSimulationSaveState_get_solid_cells
        pb.init_lib_func(libfunc, 
                         [c_void_p, c_int, c_int, c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), startidx, endidx, out])

        return out

    @_check_load_state_initialized
    def is_fluid_brick_grid_enabled(self):
        libfunc = lib.FluidSimulationSaveState_is_fluid_brick_grid_enabled
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    def is_load_state_initialized(self):
        libfunc = lib.FluidSimulationSaveState_is_load_state_initialized
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

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

    def _check_load_state(self):
        if not self.is_load_state_initialized():
            raise RuntimeError("Savestate must be loaded to use this method.")
